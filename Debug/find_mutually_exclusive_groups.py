import json
import os
from typing import List, Dict, Set, Tuple

def find_mutually_exclusive_groups(json_file: str) -> Tuple[List[List[int]], List[str]]:
    """
    Find mutually exclusive groups from agent settings in JSON file.
    
    Args:
        json_file (str): Path to the JSON file containing agent settings
        
    Returns:
        Tuple[List[List[int]], List[str]]: Tuple containing:
            - List of mutually exclusive groups (each group is a list of agent IDs)
            - List of error messages for circular relationships
    """
    # Check if file exists
    if not os.path.exists(json_file):
        raise FileNotFoundError(f"JSON file not found: {json_file}")
    
    # Read JSON file
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON file: {e}")
    
    # Store neighbor relationships
    agents: Dict[str, List[int]] = {}
    for item in data:
        agent_id = str(int(item['ID']))  # Convert to integer and back to string to remove decimal
        neighbors = item['neighbour_v_id']
        # Convert single value to list
        if not isinstance(neighbors, list):
            neighbors = [neighbors]
        # Convert to integers
        neighbors = [int(n) for n in neighbors]
        agents[agent_id] = neighbors
    
    # Initialize tracking variables
    errors: List[str] = []
    meg: List[List[int]] = []
    visited: Set[str] = set()
    in_stack: Set[str] = set()
    
    def dfs(agent: str, group: List[int]) -> None:
        """Depth-first search to find connected components and detect cycles."""
        visited.add(agent)
        in_stack.add(agent)
        group.append(int(agent))
        
        for neighbor in agents[agent]:
            neighbor_str = str(neighbor)
            if neighbor_str not in visited:
                dfs(neighbor_str, group)
            elif neighbor_str in in_stack:
                errors.append(f"Circular relationship detected: {agent} <-> {neighbor_str}")
        
        in_stack.remove(agent)
    
    # Run DFS for all agents
    for agent in agents:
        if agent not in visited:
            group = []
            dfs(agent, group)
            if group:
                meg.append(group)
    
    return meg, errors

if __name__ == "__main__":
    try:
        # Get the directory where the script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the path to the JSON file
        json_file = os.path.join(script_dir, "agentsetting.json")
        
        print(f"Looking for JSON file at: {json_file}")
        
        meg, errors = find_mutually_exclusive_groups(json_file)
        
        print("\nMutually Exclusive Groups:")
        if not meg:
            print("No groups found.")
        else:
            for i, group in enumerate(meg, 1):
                print(f"Group {i}: {group}")
        
        if errors:
            print("\nErrors:")
            for error in errors:
                print(error)
        else:
            print("\nNo circular relationships found.")
            
    except Exception as e:
        print(f"Error: {str(e)}")
        print("\nPlease make sure:")
        print("1. The script is in the same directory as agentsetting.json")
        print("2. The JSON file is properly formatted")
        print("3. You have read permissions for the file") 