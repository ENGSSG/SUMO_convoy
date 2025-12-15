#!/usr/bin/env python3
"""
Progressive Convoy Handoff - With Proper Platooning
====================================================
Key features:
1. Uses SUMO's CACC (Cooperative Adaptive Cruise Control) for realistic following
2. Malfunction vehicle visually follows behind helper in tight formation
3. Helper slows down and waits if malfunction falls behind
4. Clean handoff transitions between helpers

The scenario:
- Malfunction vehicle at A0 wants to reach D3 (diagonal across grid)
- It cannot plan routes, but CAN follow another vehicle
- Helpers traveling through the grid offer assistance based on route overlap
- When a helper's route diverges, handoff to the next helper
"""

import os
import sys
import math
import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple
from enum import Enum

# SUMO TraCI setup
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    os.environ['SUMO_HOME'] = '/usr/share/sumo'
    sys.path.append('/usr/share/sumo/tools')

import traci

# ============================================================================
# Configuration
# ============================================================================

class Config:
    # Platoon parameters
    PLATOON_GAP = 8.0           # Target gap between vehicles (meters)
    PLATOON_SPEED = 12.0        # Platoon cruising speed (m/s) ~43 km/h
    CATCHUP_SPEED = 14.0        # Speed when malfunction needs to catch up
    WAIT_SPEED = 4.0            # Helper slows to this when waiting
    MAX_GAP_BEFORE_WAIT = 25.0  # If gap exceeds this, helper waits
    
    # Handoff parameters  
    HANDOFF_OVERLAP_THRESHOLD = 2  # Trigger handoff when this few edges overlap remain
    
    # Simulation
    MALFUNCTION_START_EDGE = "A0B0"
    MALFUNCTION_DESTINATION = "D3"
    
    # Timing
    MALFUNCTION_SPAWN_TIME = 8.0
    HELPER_SPAWN_INTERVAL = 10.0
    FIRST_HELPER_TIME = 2.0

# ============================================================================
# Data Structures  
# ============================================================================

class VehicleRole(Enum):
    MALFUNCTION = "malfunction"
    HELPER = "helper"
    BACKGROUND = "background"

class EscortState(Enum):
    IDLE = "idle"
    REQUESTING = "requesting"
    BEING_ESCORTED = "being_escorted"
    ARRIVED = "arrived"

@dataclass
class HelperInfo:
    vehicle_id: str
    destination_edge: str
    route_edges: List[str]
    is_escorting: bool = False
    original_color: Tuple[int, int, int, int] = (50, 200, 50, 255)
    
@dataclass
class EscortSession:
    malfunction_id: str
    helper_id: str
    start_time: float
    start_edge: str

# ============================================================================
# Utility Functions
# ============================================================================

def compute_route_overlap(route_a: List[str], route_b: List[str]) -> List[str]:
    """Find common edges between two routes, preserving order from route_a"""
    set_b = set(route_b)
    return [e for e in route_a if e in set_b]

def get_remaining_route(vehicle_id: str) -> List[str]:
    """Get the remaining edges in a vehicle's route from current position"""
    try:
        route = list(traci.vehicle.getRoute(vehicle_id))
        current_index = traci.vehicle.getRouteIndex(vehicle_id)
        return route[current_index:]
    except:
        return []

def distance_between_vehicles(veh1: str, veh2: str) -> float:
    """Calculate Euclidean distance between two vehicles"""
    try:
        pos1 = traci.vehicle.getPosition(veh1)
        pos2 = traci.vehicle.getPosition(veh2)
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    except:
        return float('inf')

def get_leader_gap(follower_id: str, leader_id: str) -> float:
    """Get the gap between follower and leader along the road"""
    try:
        # Use getLeader if they're on same lane
        leader_info = traci.vehicle.getLeader(follower_id, 100)
        if leader_info and leader_info[0] == leader_id:
            return leader_info[1]  # Returns gap distance
        # Fallback to Euclidean distance
        return distance_between_vehicles(follower_id, leader_id)
    except:
        return float('inf')

# ============================================================================
# Main Escort Coordinator
# ============================================================================

class EscortCoordinator:
    def __init__(self):
        self.malfunction_id: Optional[str] = None
        self.malfunction_state = EscortState.IDLE
        self.malfunction_destination = Config.MALFUNCTION_DESTINATION
        self.destination_route: List[str] = []
        
        self.helpers: Dict[str, HelperInfo] = {}
        self.active_escort: Optional[EscortSession] = None
        
        self.events: List[Dict] = []
        self.helper_counter = 0
        
    def log_event(self, event_type: str, data: Dict):
        """Log event for analysis"""
        event = {
            "time": traci.simulation.getTime(),
            "type": event_type,
            **data
        }
        self.events.append(event)
        print(f"[{event['time']:6.1f}s] {event_type}: {data}")
        
    # -------------------------------------------------------------------------
    # Vehicle Spawning
    # -------------------------------------------------------------------------
    
    def spawn_malfunction_vehicle(self):
        """Spawn the malfunctioning vehicle with CACC car-following model"""
        vid = "malfunction_0"
        
        # Define route to destination
        self.destination_route = ["A0B0", "B0B1", "B1B2", "B2B3", "B3C3", "C3D3"]
        
        traci.route.add(f"route_{vid}", self.destination_route)
        traci.vehicle.add(vid, f"route_{vid}", typeID="malfunction")
        
        # Configure for platooning
        traci.vehicle.setColor(vid, (255, 50, 50, 255))  # Bright red
        traci.vehicle.setSpeedMode(vid, 0)  # Full control over speed
        traci.vehicle.setLaneChangeMode(vid, 0)  # Disable lane changes
        traci.vehicle.setSpeed(vid, 0)  # Start stationary, waiting for helper
        
        # Set car-following parameters for tight following
        traci.vehicle.setMinGap(vid, 2.0)
        traci.vehicle.setTau(vid, 0.5)  # Faster reaction time
        
        self.malfunction_id = vid
        self.malfunction_state = EscortState.REQUESTING
        
        self.log_event("MALFUNCTION_SPAWN", {
            "vehicle": vid,
            "destination": self.malfunction_destination,
            "route": self.destination_route
        })
        
    def spawn_helper_vehicle(self):
        """Spawn a helper vehicle"""
        self.helper_counter += 1
        vid = f"helper_{self.helper_counter}"
        
        # Diverse helper routes - mix of full path and partial
        helper_routes = [
            # Full path helpers - START ON SAME EDGE as malfunction
            ["A0B0", "B0B1", "B1B2", "B2B3", "B3C3", "C3D3"],  # Exact match - best helper
            ["A0B0", "B0B1", "B1B2", "B2B3", "B3C3", "C3D3"],  # Duplicate for frequency
            
            # Partial helpers (good for demonstrating handoffs)  
            ["A0B0", "B0B1", "B1B2", "B2C2", "C2D2"],  # Diverges at B2
            ["B0B1", "B1B2", "B2B3", "B3C3", "C3D3"],  # Joins at B0
            ["B1B2", "B2B3", "B3C3", "C3D3"],  # Joins at B1
            ["B2B3", "B3C3", "C3D3"],  # Final segment only
            
            # Alternative full paths
            ["A0B0", "B0C0", "C0C1", "C1C2", "C2C3", "C3D3"],  # Goes east first
        ]
        
        route_edges = helper_routes[self.helper_counter % len(helper_routes)]
        route_id = f"route_{vid}"
        
        try:
            traci.route.add(route_id, route_edges)
            traci.vehicle.add(vid, route_id, typeID="helper")
            
            # Configure helper
            traci.vehicle.setColor(vid, (50, 200, 50, 255))  # Green
            traci.vehicle.setSpeedMode(vid, 0)  # Full speed control
            traci.vehicle.setSpeed(vid, Config.PLATOON_SPEED)
            
            self.helpers[vid] = HelperInfo(
                vehicle_id=vid,
                destination_edge=route_edges[-1],
                route_edges=route_edges,
                is_escorting=False
            )
            
            self.log_event("HELPER_SPAWN", {
                "vehicle": vid,
                "route": route_edges
            })
        except Exception as e:
            print(f"Failed to spawn helper: {e}")
    
    # -------------------------------------------------------------------------
    # Escort Logic
    # -------------------------------------------------------------------------
    
    def find_best_helper(self) -> Optional[str]:
        """Find the best available helper based on route overlap"""
        if not self.destination_route:
            return None
            
        best_helper = None
        best_overlap = 0
        best_distance = float('inf')
        
        active_vehicles = traci.vehicle.getIDList()
        mal_pos = traci.vehicle.getPosition(self.malfunction_id) if self.malfunction_id in active_vehicles else None
        
        for vid, helper in self.helpers.items():
            if helper.is_escorting or vid not in active_vehicles:
                continue
            
            helper_remaining = get_remaining_route(vid)
            if not helper_remaining:
                continue
                
            overlap = compute_route_overlap(self.destination_route, helper_remaining)
            overlap_count = len(overlap)
            
            # Also consider distance to malfunction vehicle
            if mal_pos:
                helper_pos = traci.vehicle.getPosition(vid)
                dist = math.sqrt((mal_pos[0] - helper_pos[0])**2 + (mal_pos[1] - helper_pos[1])**2)
            else:
                dist = float('inf')
            
            # Prefer helpers with more overlap, break ties by distance
            if overlap_count > best_overlap or (overlap_count == best_overlap and dist < best_distance):
                best_overlap = overlap_count
                best_helper = vid
                best_distance = dist
                
        if best_helper and best_overlap >= 1:
            return best_helper
        return None
    
    def start_escort(self, helper_id: str):
        """Begin escort relationship with proper platooning setup"""
        if self.malfunction_id not in traci.vehicle.getIDList():
            return
            
        helper = self.helpers.get(helper_id)
        if not helper:
            return
        
        # Mark helper as escorting
        helper.is_escorting = True
        traci.vehicle.setColor(helper_id, (0, 255, 0, 255))  # Bright green = active escort
        
        # Create escort session
        self.active_escort = EscortSession(
            malfunction_id=self.malfunction_id,
            helper_id=helper_id,
            start_time=traci.simulation.getTime(),
            start_edge=traci.vehicle.getRoadID(self.malfunction_id)
        )
        
        self.malfunction_state = EscortState.BEING_ESCORTED
        
        # Sync malfunction route to follow helper
        self._sync_route_to_helper(helper_id)
        
        self.log_event("ESCORT_START", {
            "malfunction": self.malfunction_id,
            "helper": helper_id
        })
    
    def _sync_route_to_helper(self, helper_id: str):
        """Update malfunction vehicle's route to match helper's remaining route"""
        if self.malfunction_id not in traci.vehicle.getIDList():
            return
        if helper_id not in traci.vehicle.getIDList():
            return
            
        try:
            helper_route = get_remaining_route(helper_id)
            if not helper_route:
                return
            
            mal_edge = traci.vehicle.getRoadID(self.malfunction_id)
            if mal_edge.startswith(':'):
                # In junction, get next edge
                route = traci.vehicle.getRoute(self.malfunction_id)
                idx = traci.vehicle.getRouteIndex(self.malfunction_id)
                if idx < len(route):
                    mal_edge = route[idx]
            
            # Build route from current position through helper's route
            if mal_edge in helper_route:
                new_route = helper_route[helper_route.index(mal_edge):]
            else:
                # Find connecting route
                try:
                    connection = traci.simulation.findRoute(mal_edge, helper_route[0])
                    new_route = list(connection.edges) + helper_route[1:]
                except:
                    new_route = helper_route
            
            if new_route and len(new_route) > 0:
                traci.vehicle.setRoute(self.malfunction_id, new_route)
                
        except Exception as e:
            pass  # Silently handle route sync errors
    
    def check_handoff_needed(self) -> bool:
        """Check if handoff is needed"""
        if not self.active_escort:
            return False
            
        helper_id = self.active_escort.helper_id
        
        # Helper left simulation
        if helper_id not in traci.vehicle.getIDList():
            return True
        
        # Check remaining overlap
        helper_remaining = get_remaining_route(helper_id)
        overlap = compute_route_overlap(self.destination_route, helper_remaining)
        
        return len(overlap) <= Config.HANDOFF_OVERLAP_THRESHOLD
    
    def execute_handoff(self, new_helper_id: str):
        """Transfer escort to new helper"""
        old_helper_id = self.active_escort.helper_id if self.active_escort else None
        
        if old_helper_id and old_helper_id in self.helpers:
            self.helpers[old_helper_id].is_escorting = False
            if old_helper_id in traci.vehicle.getIDList():
                # Restore normal color and speed
                traci.vehicle.setColor(old_helper_id, (100, 180, 100, 255))
                traci.vehicle.setSpeedMode(old_helper_id, 31)  # Normal mode
                traci.vehicle.setSpeed(old_helper_id, -1)  # Let SUMO control
        
        self.log_event("HANDOFF", {
            "from": old_helper_id,
            "to": new_helper_id
        })
        
        self.start_escort(new_helper_id)
    
    # -------------------------------------------------------------------------
    # Platooning Control - The Key Part!
    # -------------------------------------------------------------------------
    
    def update_platoon_control(self):
        """
        Main platooning control loop.
        - Helper maintains steady speed, slows if follower falls behind
        - Malfunction vehicle accelerates/decelerates to maintain gap
        """
        if not self.active_escort:
            return
            
        helper_id = self.active_escort.helper_id
        mal_id = self.malfunction_id
        
        if mal_id not in traci.vehicle.getIDList():
            return
        if helper_id not in traci.vehicle.getIDList():
            # Helper gone
            self.active_escort = None
            self.malfunction_state = EscortState.REQUESTING
            return
        
        # Get current states
        gap = distance_between_vehicles(mal_id, helper_id)
        helper_speed = traci.vehicle.getSpeed(helper_id)
        mal_speed = traci.vehicle.getSpeed(mal_id)
        
        # Get positions for direction check
        helper_pos = traci.vehicle.getPosition(helper_id)
        mal_pos = traci.vehicle.getPosition(mal_id)
        
        # ---- HELPER SPEED CONTROL ----
        # Helper slows down if malfunction is too far behind
        if gap > Config.MAX_GAP_BEFORE_WAIT:
            # Malfunction is very far, slow way down
            target_helper_speed = Config.WAIT_SPEED
        elif gap > Config.PLATOON_GAP * 2:
            # Malfunction falling behind, slow down
            target_helper_speed = Config.PLATOON_SPEED * 0.7
        else:
            # Good gap, cruise normally
            target_helper_speed = Config.PLATOON_SPEED
        
        traci.vehicle.setSpeed(helper_id, target_helper_speed)
        
        # ---- MALFUNCTION SPEED CONTROL ----
        # Use simple CACC-like control
        gap_error = gap - Config.PLATOON_GAP
        
        if gap_error > 15:
            # Very far behind, max speed to catch up
            target_mal_speed = Config.CATCHUP_SPEED
        elif gap_error > 5:
            # Behind, speed up
            target_mal_speed = helper_speed + 2.0
        elif gap_error < -3:
            # Too close, slow down
            target_mal_speed = max(1.0, helper_speed - 3.0)
        else:
            # Good gap, match leader speed
            target_mal_speed = helper_speed
        
        # Clamp speed
        target_mal_speed = max(0.5, min(Config.CATCHUP_SPEED, target_mal_speed))
        traci.vehicle.setSpeed(mal_id, target_mal_speed)
        
        # ---- ROUTE SYNC ----
        # Periodically ensure malfunction is following helper's route
        if int(traci.simulation.getTime() * 10) % 30 == 0:
            self._sync_route_to_helper(helper_id)

        # print(f"Gap: {gap:.1f}m, Helper speed: {helper_speed:.1f}, Mal speed: {target_mal_speed:.1f}")
    
    # -------------------------------------------------------------------------
    # Main Loop
    # -------------------------------------------------------------------------
    
    def step(self) -> bool:
        """Process one simulation step. Returns True if destination reached."""
        current_time = traci.simulation.getTime()
        
        # DEBUG: Print malfunction vehicle location
        if self.malfunction_id and self.malfunction_id in traci.vehicle.getIDList():
            pos = traci.vehicle.getPosition(self.malfunction_id)
            print(f"DEBUG: Malfunction {self.malfunction_id} pos: {pos}")

        
        # Check if malfunction vehicle reached destination
        if self.malfunction_id and self.malfunction_id in traci.vehicle.getIDList():
            mal_edge = traci.vehicle.getRoadID(self.malfunction_id)
            # Check for destination - edge should end at D3 or contain D3
            if mal_edge and ("D3" in mal_edge or mal_edge == "C3D3"):
                self.malfunction_state = EscortState.ARRIVED
                self.log_event("DESTINATION_REACHED", {
                    "vehicle": self.malfunction_id,
                    "edge": mal_edge,
                    "time": current_time
                })
                return True
        
        # State machine
        if self.malfunction_state == EscortState.REQUESTING:
            helper = self.find_best_helper()
            if helper:
                self.start_escort(helper)
            else:
                # No helper yet, keep malfunction vehicle slow/stopped
                if self.malfunction_id in traci.vehicle.getIDList():
                    traci.vehicle.setSpeed(self.malfunction_id, 2.0)  # Creep slowly
        
        elif self.malfunction_state == EscortState.BEING_ESCORTED:
            # Check handoff
            if self.check_handoff_needed():
                new_helper = self.find_best_helper()
                if new_helper:
                    self.execute_handoff(new_helper)
                # If no new helper, keep current one until it leaves
            
            # Update platoon control
            self.update_platoon_control()
        
        return False

# ============================================================================
# Main Simulation Runner
# ============================================================================

def run_simulation(gui=False):
    """Run the escort handoff demonstration"""
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    sumo_binary = "sumo-gui" if gui else "sumo"
    sumo_cmd = [
        sumo_binary,
        "-c", "simulation.sumocfg",
        "--step-length", "0.1",
        "--collision.action", "warn",
        "--no-warnings", "true",
        "--time-to-teleport", "-1",
    ]
    
    if gui:
        sumo_cmd.extend([
            "--start",
            "--delay", "100",
            "--quit-on-end", "false",
        ])
    
    traci.start(sumo_cmd)
    
    coordinator = EscortCoordinator()
    
    print("\n" + "="*70)
    print("Progressive Convoy Handoff - Platooning Demo")
    print("="*70)
    print(f"🔴 Red vehicle: Malfunctioning AV (follows helper)")
    print(f"🟢 Green vehicle: Helper (bright = escorting, dim = available)")
    print(f"Route: A0 → D3 (diagonal across 4x4 grid)")
    print("="*70 + "\n")
    
    last_helper_spawn = -Config.HELPER_SPAWN_INTERVAL + Config.FIRST_HELPER_TIME
    step = 0
    max_steps = 3000  # 300 seconds max
    
    while step < max_steps:
        traci.simulationStep()
        current_time = traci.simulation.getTime()
        
        # Spawn malfunction vehicle
        if current_time >= Config.MALFUNCTION_SPAWN_TIME and coordinator.malfunction_id is None:
            coordinator.spawn_malfunction_vehicle()
            traci.simulationStep()  # Let it appear
            coordinator.step()
        
        # Spawn helpers periodically
        if current_time - last_helper_spawn >= Config.HELPER_SPAWN_INTERVAL:
            coordinator.spawn_helper_vehicle()
            last_helper_spawn = current_time

        # Process escort logic
        if coordinator.malfunction_id:
            completed = coordinator.step()
            if completed:
                print("\n🎉 SUCCESS! Malfunction vehicle reached destination!")
                # Let simulation run a bit more to see the result
                for _ in range(50):
                    traci.simulationStep()
                break
        
        # Check if malfunction vehicle left simulation unexpectedly
        if (coordinator.malfunction_id and 
            coordinator.malfunction_state not in [EscortState.ARRIVED, EscortState.IDLE] and
            coordinator.malfunction_id not in traci.vehicle.getIDList()):
            print("\n❌ Malfunction vehicle left simulation unexpectedly")
            break
        
        step += 1
    
    # Summary
    print("\n" + "="*70)
    print("Simulation Summary")
    print("="*70)
    
    escort_starts = [e for e in coordinator.events if e["type"] == "ESCORT_START"]
    handoffs = [e for e in coordinator.events if e["type"] == "HANDOFF"]
    
    print(f"Escort initiations: {len(escort_starts)}")
    print(f"Handoffs: {len(handoffs)}")
    print(f"Final state: {coordinator.malfunction_state.value}")
    print(f"Simulation time: {traci.simulation.getTime():.1f}s")
    
    print("\nKey Events:")
    for event in coordinator.events:
        if event["type"] in ["MALFUNCTION_SPAWN", "ESCORT_START", "HANDOFF", "DESTINATION_REACHED"]:
            print(f"  [{event['time']:6.1f}s] {event['type']}")
    
    # Save results
    with open("simulation_results.json", "w") as f:
        json.dump({
            "events": coordinator.events,
            "summary": {
                "escort_count": len(escort_starts),
                "handoff_count": len(handoffs),
                "final_state": coordinator.malfunction_state.value,
                "total_time": traci.simulation.getTime()
            }
        }, f, indent=2)
    
    if gui:
        print("\nSimulation complete. Close SUMO window to exit.")
        try:
            while True:
                traci.simulationStep()
        except:
            pass
    
    traci.close()

# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Progressive Convoy Handoff - Platooning Demo")
    parser.add_argument("--gui", action="store_true", help="Run with SUMO GUI")
    args = parser.parse_args()
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    print(f"Working directory: {script_dir}")
    
    required_files = ["simulation.sumocfg", "network.net.xml", "routes.rou.xml"]
    missing = [f for f in required_files if not os.path.exists(f)]
    if missing:
        print(f"ERROR: Missing files: {missing}")
        print("\nRun setup first:")
        print("  ./setup.sh")
        sys.exit(1)
    
    run_simulation(gui=args.gui)
