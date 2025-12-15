#!/usr/bin/env python3
"""
Progressive Convoy Handoff Demo for SUMO
=========================================
Demonstrates: A malfunctioning AV is escorted by sequential helpers,
with handoffs occurring when route overlap decreases.

Key concepts:
- Malfunctioning vehicle follows helper via car-following
- Helpers bid based on route overlap utility
- Handoff triggered when current helper's remaining overlap < threshold
"""

import os
import sys
import math
import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
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
# Data structures
# ============================================================================

class VehicleState(Enum):
    NORMAL = "normal"
    MALFUNCTIONING = "malfunctioning"
    ESCORTING = "escorting"
    BEING_ESCORTED = "being_escorted"
    HANDOFF_PENDING = "handoff_pending"

@dataclass
class Vehicle:
    """Represents a vehicle in the simulation"""
    id: str
    vtype: str
    route: str
    state: VehicleState = VehicleState.NORMAL
    helper_id: Optional[str] = None
    escortee_id: Optional[str] = None
    route_edges: List[str] = field(default_factory=list)
    
@dataclass
class EscortRequest:
    """Broadcast when a vehicle malfunctions"""
    vehicle_id: str
    current_edge: str
    destination_edges: List[str]
    timestamp: float

@dataclass 
class HelperBid:
    """Response to escort request"""
    helper_id: str
    overlap_edges: List[str]
    utility: float  # Based on overlap length and detour cost
    
# ============================================================================
# Core algorithms
# ============================================================================

def compute_route_overlap(route_a: List[str], route_b: List[str]) -> List[str]:
    """Find common edges between two routes (preserving order from route_a)"""
    set_b = set(route_b)
    return [e for e in route_a if e in set_b]

def compute_remaining_overlap(vehicle_edge: str, helper_route: List[str], 
                               malfunction_route: List[str]) -> List[str]:
    """Compute remaining overlap from current position"""
    try:
        helper_idx = helper_route.index(vehicle_edge)
        remaining_helper = helper_route[helper_idx:]
    except ValueError:
        remaining_helper = helper_route
    
    return compute_route_overlap(remaining_helper, malfunction_route)

def calculate_bid_utility(overlap_edges: List[str], helper_route: List[str],
                          edge_lengths: Dict[str, float]) -> float:
    """
    Utility function for helper selection.
    Higher utility = better helper choice.
    
    Factors:
    - Overlap length (positive)
    - Detour required (negative) 
    """
    overlap_length = sum(edge_lengths.get(e, 100) for e in overlap_edges)
    total_route_length = sum(edge_lengths.get(e, 100) for e in helper_route)
    
    # Simple utility: overlap as fraction of helper's remaining route
    # A helper whose entire route overlaps is ideal
    if total_route_length == 0:
        return 0
    
    utility = overlap_length / total_route_length
    return utility

# ============================================================================
# Escort Protocol Implementation  
# ============================================================================

class EscortCoordinator:
    """Manages the escort and handoff protocol"""
    
    def __init__(self, handoff_threshold: float = 100.0):
        self.handoff_threshold = handoff_threshold  # meters of remaining overlap
        self.vehicles: Dict[str, Vehicle] = {}
        self.active_escorts: Dict[str, str] = {}  # malfunction_id -> helper_id
        self.pending_requests: List[EscortRequest] = []
        self.edge_lengths: Dict[str, float] = {}
        self.events: List[Dict] = []  # Log of events for analysis
        
    def register_vehicle(self, vid: str, vtype: str, route_edges: List[str]):
        """Register a vehicle in the system"""
        state = VehicleState.MALFUNCTIONING if vtype == "malfunction" else VehicleState.NORMAL
        self.vehicles[vid] = Vehicle(
            id=vid, vtype=vtype, route=route_edges[0] if route_edges else "",
            state=state, route_edges=route_edges
        )
        
    def cache_edge_lengths(self):
        """Cache edge lengths from SUMO"""
        for edge_id in traci.edge.getIDList():
            self.edge_lengths[edge_id] = traci.lane.getLength(f"{edge_id}_0")
    
    def broadcast_escort_request(self, malfunction_id: str):
        """Malfunctioning vehicle broadcasts request for help"""
        vehicle = self.vehicles.get(malfunction_id)
        if not vehicle:
            return
            
        current_edge = traci.vehicle.getRoadID(malfunction_id)
        request = EscortRequest(
            vehicle_id=malfunction_id,
            current_edge=current_edge,
            destination_edges=vehicle.route_edges,
            timestamp=traci.simulation.getTime()
        )
        self.pending_requests.append(request)
        self.log_event("ESCORT_REQUEST", malfunction_id, {"edge": current_edge})
        print(f"[{traci.simulation.getTime():.1f}s] 🚨 {malfunction_id} broadcasts escort request at {current_edge}")
        
    def collect_bids(self, request: EscortRequest) -> List[HelperBid]:
        """Collect bids from nearby operational vehicles"""
        bids = []
        
        for vid, vehicle in self.vehicles.items():
            # Skip if not available to help
            if (vehicle.vtype != "helper" or 
                vehicle.state in [VehicleState.ESCORTING, VehicleState.MALFUNCTIONING]):
                continue
            
            # Check if vehicle is in simulation
            if vid not in traci.vehicle.getIDList():
                continue
                
            # Compute route overlap
            overlap = compute_route_overlap(vehicle.route_edges, request.destination_edges)
            
            if len(overlap) > 0:
                utility = calculate_bid_utility(
                    overlap, vehicle.route_edges, self.edge_lengths
                )
                bids.append(HelperBid(
                    helper_id=vid,
                    overlap_edges=overlap,
                    utility=utility
                ))
                print(f"  📋 {vid} bids with utility {utility:.2f} ({len(overlap)} overlapping edges)")
        
        return bids
    
    def select_helper(self, bids: List[HelperBid]) -> Optional[HelperBid]:
        """Select best helper via auction (highest utility wins)"""
        if not bids:
            return None
        return max(bids, key=lambda b: b.utility)
    
    def initiate_escort(self, malfunction_id: str, helper_id: str):
        """Start escort relationship"""
        self.vehicles[malfunction_id].state = VehicleState.BEING_ESCORTED
        self.vehicles[malfunction_id].helper_id = helper_id
        self.vehicles[helper_id].state = VehicleState.ESCORTING
        self.vehicles[helper_id].escortee_id = malfunction_id
        self.active_escorts[malfunction_id] = helper_id
        
        self.log_event("ESCORT_START", malfunction_id, {"helper": helper_id})
        print(f"[{traci.simulation.getTime():.1f}s] ✅ {helper_id} begins escorting {malfunction_id}")
        
        # Visual feedback
        traci.vehicle.setColor(helper_id, (0, 255, 0, 255))  # Bright green
        
    def check_handoff_needed(self, malfunction_id: str) -> bool:
        """Check if current helper's remaining overlap is below threshold"""
        helper_id = self.active_escorts.get(malfunction_id)
        if not helper_id or helper_id not in traci.vehicle.getIDList():
            return True  # Helper gone, need new one
            
        helper = self.vehicles[helper_id]
        malfunction = self.vehicles[malfunction_id]
        
        try:
            helper_edge = traci.vehicle.getRoadID(helper_id)
            remaining_overlap = compute_remaining_overlap(
                helper_edge, helper.route_edges, malfunction.route_edges
            )
            remaining_length = sum(
                self.edge_lengths.get(e, 100) for e in remaining_overlap
            )
            
            return remaining_length < self.handoff_threshold
        except:
            return True
    
    def execute_handoff(self, malfunction_id: str, new_helper_id: str):
        """Transfer escort responsibility to new helper"""
        old_helper_id = self.active_escorts.get(malfunction_id)
        
        if old_helper_id and old_helper_id in self.vehicles:
            self.vehicles[old_helper_id].state = VehicleState.NORMAL
            self.vehicles[old_helper_id].escortee_id = None
            if old_helper_id in traci.vehicle.getIDList():
                traci.vehicle.setColor(old_helper_id, (100, 200, 100, 255))  # Dim green
        
        self.initiate_escort(malfunction_id, new_helper_id)
        self.log_event("HANDOFF", malfunction_id, {
            "old_helper": old_helper_id, "new_helper": new_helper_id
        })
        print(f"[{traci.simulation.getTime():.1f}s] 🔄 HANDOFF: {old_helper_id} → {new_helper_id}")
        
    def update_escort_behavior(self):
        """Update vehicle behaviors based on escort relationships"""
        for malfunction_id, helper_id in list(self.active_escorts.items()):
            if malfunction_id not in traci.vehicle.getIDList():
                continue
            if helper_id not in traci.vehicle.getIDList():
                # Helper left, need new one
                del self.active_escorts[malfunction_id]
                self.vehicles[malfunction_id].state = VehicleState.MALFUNCTIONING
                self.broadcast_escort_request(malfunction_id)
                continue
                
            # Make malfunction vehicle follow helper
            try:
                helper_pos = traci.vehicle.getPosition(helper_id)
                helper_speed = traci.vehicle.getSpeed(helper_id)
                
                # Adjust malfunction vehicle to follow
                traci.vehicle.setSpeedMode(malfunction_id, 0)  # Disable all checks
                
                # Calculate following distance
                mal_pos = traci.vehicle.getPosition(malfunction_id)
                distance = math.sqrt((helper_pos[0] - mal_pos[0])**2 + 
                                    (helper_pos[1] - mal_pos[1])**2)
                
                # Speed control: slow down if too close, speed up if too far
                desired_gap = 15.0  # meters
                if distance < desired_gap - 5:
                    target_speed = max(0, helper_speed - 2)
                elif distance > desired_gap + 5:
                    target_speed = min(8, helper_speed + 1)  # Max 8 m/s for malfunction
                else:
                    target_speed = min(8, helper_speed)
                    
                traci.vehicle.setSpeed(malfunction_id, target_speed)
                
            except traci.TraCIException:
                pass
    
    def log_event(self, event_type: str, vehicle_id: str, data: Dict):
        """Log event for analysis"""
        self.events.append({
            "time": traci.simulation.getTime(),
            "type": event_type,
            "vehicle": vehicle_id,
            "data": data
        })
    
    def process_step(self):
        """Main processing loop - called each simulation step"""
        current_time = traci.simulation.getTime()
        
        # Process pending escort requests
        for request in self.pending_requests[:]:
            if request.vehicle_id not in traci.vehicle.getIDList():
                self.pending_requests.remove(request)
                continue
                
            bids = self.collect_bids(request)
            winner = self.select_helper(bids)
            
            if winner:
                self.initiate_escort(request.vehicle_id, winner.helper_id)
                self.pending_requests.remove(request)
            else:
                # No helper available yet, will retry
                pass
        
        # Check for handoff conditions
        for malfunction_id in list(self.active_escorts.keys()):
            if malfunction_id not in traci.vehicle.getIDList():
                del self.active_escorts[malfunction_id]
                continue
                
            if self.check_handoff_needed(malfunction_id):
                self.vehicles[malfunction_id].state = VehicleState.HANDOFF_PENDING
                # Create new request for handoff
                request = EscortRequest(
                    vehicle_id=malfunction_id,
                    current_edge=traci.vehicle.getRoadID(malfunction_id),
                    destination_edges=self.vehicles[malfunction_id].route_edges,
                    timestamp=current_time
                )
                bids = self.collect_bids(request)
                winner = self.select_helper(bids)
                
                if winner:
                    self.execute_handoff(malfunction_id, winner.helper_id)
        
        # Update escort following behavior
        self.update_escort_behavior()

# ============================================================================
# Main simulation
# ============================================================================

def run_simulation(gui=False):
    """Run the escort handoff demonstration
    
    Args:
        gui: If True, run with SUMO GUI for visualization
    """
    
    # Start SUMO
    sumo_binary = "sumo-gui" if gui else "sumo"
    sumo_cmd = [
        sumo_binary,
        "-c", "simulation.sumocfg",
        "--step-length", "0.1",
        "--collision.action", "warn",
        "--no-warnings", "true",
    ]
    
    if gui:
        # GUI-specific options
        sumo_cmd.extend([
            "--start",  # Start simulation immediately
            "--delay", "100",  # Slow down for visibility (ms per step)
            "--quit-on-end", "true",
        ])
    
    traci.start(sumo_cmd)
    
    # Initialize coordinator
    coordinator = EscortCoordinator(handoff_threshold=150.0)
    coordinator.cache_edge_lengths()
    
    # Define routes for vehicles
    routes = {
        "malfunction_0": ["W_J1", "J1_J2", "J2_J3", "J3_E"],
        "helper_A": ["S1_J1", "J1_J2", "J2_N2"],
        "helper_B": ["S2_J2", "J2_J3", "J3_N3"],
        "helper_C": ["S3_J3", "J3_E"],
    }
    
    # Vehicle spawn schedule (time, id, type, route, depart_edge)
    spawn_schedule = [
        (5.0, "malfunction_0", "malfunction", "route_malfunction"),
        (3.0, "helper_A", "helper", "route_helper_A"),
        (15.0, "helper_B", "helper", "route_helper_B"),
        (25.0, "helper_C", "helper", "route_helper_C"),
    ]
    
    spawned = set()
    malfunction_triggered = False
    
    print("\n" + "="*60)
    print("Progressive Convoy Handoff Simulation")
    print("="*60 + "\n")
    
    step = 0
    while step < 1000:  # Max 100 seconds
        traci.simulationStep()
        current_time = traci.simulation.getTime()
        
        # Spawn vehicles according to schedule
        for spawn_time, vid, vtype, route in spawn_schedule:
            if current_time >= spawn_time and vid not in spawned:
                try:
                    traci.vehicle.add(vid, route, typeID=vtype)
                    coordinator.register_vehicle(vid, vtype, routes[vid])
                    spawned.add(vid)
                    print(f"[{current_time:.1f}s] 🚗 {vid} ({vtype}) entered simulation")
                    
                    # Set colors
                    if vtype == "malfunction":
                        traci.vehicle.setColor(vid, (255, 0, 0, 255))  # Red
                    elif vtype == "helper":
                        traci.vehicle.setColor(vid, (100, 200, 100, 255))  # Light green
                except:
                    pass
        
        # Trigger malfunction and initial escort request
        if not malfunction_triggered and current_time >= 8.0:
            if "malfunction_0" in traci.vehicle.getIDList():
                coordinator.broadcast_escort_request("malfunction_0")
                malfunction_triggered = True
        
        # Process escort protocol
        coordinator.process_step()
        
        # Check if simulation should end
        if "malfunction_0" not in traci.vehicle.getIDList() and malfunction_triggered:
            print(f"\n[{current_time:.1f}s] 🏁 Malfunction vehicle reached destination!")
            break
            
        step += 1
    
    # Print summary
    print("\n" + "="*60)
    print("Simulation Summary")
    print("="*60)
    
    escort_events = [e for e in coordinator.events if e["type"] == "ESCORT_START"]
    handoff_events = [e for e in coordinator.events if e["type"] == "HANDOFF"]
    
    print(f"Total escort initiations: {len(escort_events)}")
    print(f"Total handoffs: {len(handoff_events)}")
    
    print("\nEvent timeline:")
    for event in coordinator.events:
        print(f"  [{event['time']:.1f}s] {event['type']}: {event['vehicle']} - {event['data']}")
    
    # Save events to file
    with open("simulation_results.json", "w") as f:
        json.dump({
            "events": coordinator.events,
            "summary": {
                "escort_count": len(escort_events),
                "handoff_count": len(handoff_events),
                "total_time": traci.simulation.getTime()
            }
        }, f, indent=2)
    
    print("\nResults saved to simulation_results.json")
    
    traci.close()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Progressive Convoy Handoff Simulation")
    parser.add_argument("--gui", action="store_true", help="Run with SUMO GUI for visualization")
    args = parser.parse_args()
    
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    print(f"Working directory: {script_dir}")
    
    # Check that required files exist
    required_files = ["simulation.sumocfg", "network.net.xml", "routes.rou.xml"]
    missing = [f for f in required_files if not os.path.exists(f)]
    if missing:
        print(f"ERROR: Missing required files: {missing}")
        print(f"Current directory contains: {os.listdir('.')}")
        print("\nYou need to generate the network file first. Run:")
        print("  netconvert --node-files=network.nod.xml --edge-files=network.edg.xml --output-file=network.net.xml")
        sys.exit(1)
    
    run_simulation(gui=args.gui)
