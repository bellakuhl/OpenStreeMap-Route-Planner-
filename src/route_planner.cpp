#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// Calculates heuristic value for A* search
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*RoutePlanner::end_node);
}


// AddNeighbors method expands the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // Populate current_node.neighbors vector with all the neighbors
    current_node->FindNeighbors();

    // For each node in current_node.neighbors, set the parent, the h_value, the g_value.
    for (auto *curr : current_node->neighbors) {
        curr->g_value = current_node->g_value + current_node->distance(*curr);
        curr->h_value = CalculateHValue(curr);
        curr->parent = current_node;

        // Add the neighbor to open_list and set the node's visited attribute to true
        this->open_list.push_back(curr);
        curr->visited = true;
    }
}


// NextNode method sorts the open list and returns the next node.

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort list according to h and g value
    std::sort(this->open_list.begin(), this->open_list.end(), [](auto &node1, auto &node2)
    {
        return (node1->g_value + node1->h_value) < (node2->g_value + node2->h_value);
    });

    RouteModel::Node* lowestSumNode = nullptr;

    lowestSumNode = open_list.front();
    open_list.erase(open_list.begin()); // Remove node with the lowest sum of g and h
    
    return lowestSumNode;
    
}


// ConstructFinalPath method returns the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // For each node, add distance from node to its parent
    while (current_node != start_node) {
        distance += current_node->distance(*(current_node->parent));
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end()); // reverse path so that its in the correct order (start->end)
    return path_found;

}


// A* Search algorithm 

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    this->start_node->visited = true; // already visited since we've started the search
    open_list.emplace_back(this->start_node);

    while(!this->open_list.empty()) {
        current_node = NextNode();
        if (current_node->distance(*this->end_node) == 0) { // if the current node is the end node, add to final path
            m_Model.path = ConstructFinalPath(current_node);
            return;
        } else {
            AddNeighbors(current_node); // otherwise, get neighbors and add to open_list
        }
    }
}