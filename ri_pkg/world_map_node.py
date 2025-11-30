import json
import networkx as nx
import matplotlib.pyplot as plt
import math
import re
import os

# --- CONFIGURATION ---
# Replace this with your actual file name
FILENAME = "../assets/detected_objects_map_iras.json"

def extract_clean_label(raw_label):
    """Parses VLM output to get a short name."""
    # Pattern 1: "Name: Object"
    match = re.search(r"Name:\s*(.*?)(?:\n|$)", raw_label)
    if match:
        return match.group(1).strip()
    
    # Pattern 2: "The main object is a [Object]."
    if "is a" in raw_label:
        parts = raw_label.split("is a")
        if len(parts) > 1:
            return parts[1].split('.')[0].split(',')[0].strip()
            
    # Fallback: Truncate if too long
    return raw_label[:15] + "..." if len(raw_label) > 15 else raw_label

def calculate_edge_attributes(node1_data, node2_data):
    """Calculates Euclidean distance between two objects."""
    x1, y1 = node1_data['world_coordinates']['x'], node1_data['world_coordinates']['y']
    x2, y2 = node2_data['world_coordinates']['x'], node2_data['world_coordinates']['y']
    
    dx = x2 - x1
    dy = y2 - y1
    
    dist = math.sqrt(dx**2 + dy**2)
    return dist

def create_scene_graph(data):
    G = nx.Graph()
    
    # --- STEP 1: ADD NODES ---
    for item in data:
        # LOGIC: Check for Human Clarified status
        # In a real file, you might have a field "is_human_verified": true
        # Here we simulate it for ID 2 as per your request
        if item.get('id') == 2: 
            display_name = "Blue Bg (HC)"
            conf = 1.0  # Force confidence to 1.0 for Human Clarified
            node_color = '#32CD32' # Lime Green
            shape = 's' # Square
        else:
            display_name = extract_clean_label(item['label'])
            conf = item['confidence']
            node_color = '#87CEEB' # Sky Blue
            shape = 'o' # Circle
            
        # Add Node using global X,Y as position
        G.add_node(item['id'], 
                   pos=(item['world_coordinates']['x'], item['world_coordinates']['y']),
                   label=f"{display_name}\n{conf}",
                   color=node_color,
                   shape=shape,
                   raw_data=item)

    # --- STEP 2: ADD EDGES (Spatial Connections) ---
    # Create a fully connected graph with distances as weights
    nodes = list(G.nodes(data=True))
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            id1, data1 = nodes[i]
            id2, data2 = nodes[j]
            
            dist = calculate_edge_attributes(data1['raw_data'], data2['raw_data'])
            
            # Add edge with distance label
            G.add_edge(id1, id2, weight=dist, label=f"{dist:.2f}m")

    return G

def visualize_graph(G):
    if not G.nodes:
        print("Graph is empty. No objects to visualize.")
        return

    pos = nx.get_node_attributes(G, 'pos')
    colors = [node[1]['color'] for node in G.nodes(data=True)]
    labels = nx.get_node_attributes(G, 'label')
    edge_labels = nx.get_edge_attributes(G, 'label')
    
    plt.figure(figsize=(10, 8))
    
    # 1. Draw Robot Origin (0,0)
    plt.plot(0, 0, marker='^', color='red', markersize=15, label='Robot Start', linestyle='None')

    # 2. Draw Nodes
    nx.draw_networkx_nodes(G, pos, node_color=colors, node_size=3500, alpha=0.9, edgecolors='black')
    
    # 3. Draw Node Labels
    nx.draw_networkx_labels(G, pos, labels, font_size=9, font_weight='bold')
    
    # 4. Draw Edges
    nx.draw_networkx_edges(G, pos, style='dashed', alpha=0.5, edge_color='gray')
    
    # 5. Draw Edge Labels (Distances)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')

    # Formatting
    plt.title(f"Spatial Object Graph loaded from {FILENAME}")
    plt.xlabel("Global X (meters)")
    plt.ylabel("Global Y (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal') # Keeps physical proportions correct
    plt.legend()
    
    # Save or Show
    plt.savefig("assets/object_graph_map_IRAS.png")
    print("Graph saved to assets/object_graph_map_IRAS.png")
    plt.show()

if __name__ == "__main__":
    if os.path.exists(FILENAME):
        try:
            with open(FILENAME, 'r') as f:
                objects_data = json.load(f)
            
            print(f"Successfully loaded {len(objects_data)} objects from {FILENAME}")
            graph = create_scene_graph(objects_data)
            visualize_graph(graph)
            
        except json.JSONDecodeError:
            print(f"Error: {FILENAME} contains invalid JSON.")
        except Exception as e:
            print(f"An error occurred: {e}")
    else:
        print(f"Error: File '{FILENAME}' not found. Please ensure the file path is correct.")