#include "a_star.hpp"

AStar::AStar(std::vector<graph_node> borders, std::vector<obstacle> obstacles) {

    for (auto& border : borders) {
       if (border.x < x_min) {
            x_min = border.x;
       } 
       if (border.x > x_max) {
            x_max = border.x;
       }
       if (border.y < y_min) {
            y_min = border.y;
       } 
       if (border.y > y_max) {
            y_max = border.y;
       }
    }
    
    x_width = round((x_max-x_min)/resolution)+1;
    y_width = round((y_max-y_min)/resolution)+1;

    obstacle_map.resize(x_width, std::vector<bool>(y_width, false));

    // create obstacle map
    for (int ix = 0; ix < x_width; ix++) {
        double x = get_xy_original(ix, x_min);
        for (int iy = 0; iy < y_width; iy++) {
            double y = get_xy_original(iy, y_min);
            for (auto& obstacle : obstacles) {
                if (obstacle.type == obstacle_type::CYLINDER) {
                    double d = hypot(obstacle.x - x, obstacle.y - y);
                    if (d <= ROBOT_RADIUS + obstacle.radius) {
                        obstacle_map[ix][iy] = true;
                        break;
                    }
                } else if (obstacle.type == obstacle_type::BOX) {
                    double dx = abs(obstacle.x - x);
                    double dy = abs(obstacle.y - y);
                    if (dx <= ROBOT_RADIUS+obstacle.dx/2.0 || dy <= ROBOT_RADIUS+obstacle.dy/2.0) {
                        obstacle_map[ix][iy] = true;
                        break;
                    }
                }
            }
        }
    }

    // add borders to obstacle map 
    std::vector<Point> border_points;
    std::vector<Point> corners;

    for (size_t i = 0; i < borders.size(); i++) {
        corners.push_back({get_xy_scaled(borders[i].x, x_min),get_xy_scaled(borders[i].y, y_min)});
    }

    std::vector<Point> line_1 = get_points_along_line(corners[1].x, corners[1].y, corners[2].x, corners[2].y);
    std::vector<Point> line_2 = get_points_along_line(corners[2].x, corners[2].y, corners[3].x, corners[3].y);
    std::vector<Point> line_3 = get_points_along_line(corners[4].x, corners[4].y, corners[5].x, corners[5].y);
    std::vector<Point> line_4 = get_points_along_line(corners[5].x, corners[5].y, corners[0].x, corners[0].y);

    // merge point vectors
    border_points.insert(border_points.end(), line_1.begin(), line_1.end());
    border_points.insert(border_points.end(), line_2.begin(), line_2.end());
    border_points.insert(border_points.end(), line_3.begin(), line_3.end());
    border_points.insert(border_points.end(), line_4.begin(), line_4.end());

    for (auto& border_point : border_points) {
        obstacle_map[border_point.x][border_point.y] = true;
    }
}

ShortestPath AStar::get_shortest_path(const graph_node& start_point, const graph_node& target_point) {
    ShortestPath shortest_path;

    AStarNode start_node = AStarNode(get_xy_scaled(start_point.x, x_min), get_xy_scaled(start_point.y, y_min));
    Point target_point_scaled = {get_xy_scaled(target_point.x, x_min), get_xy_scaled(target_point.y, y_min)};

    std::unordered_map<int, AStarNode> open_set_map, closed_set;

    auto compare_nodes = [&](const int& lhs, const int& rhs) {
        return open_set_map.at(lhs).getF_score(target_point_scaled) > open_set_map.at(rhs).getF_score(target_point_scaled);
    };
    std::priority_queue<int, std::vector<int>, decltype(compare_nodes)> open_set(compare_nodes);

    open_set.push(get_node_index(start_node));
    open_set_map.insert({get_node_index(start_node), start_node});

    while (!open_set.empty()) {
        AStarNode current_node = open_set_map.at(open_set.top());
        open_set.pop();
        open_set_map.erase(get_node_index(current_node));

        if (current_node == target_point_scaled) {
            shortest_path.length = current_node.g*resolution;

            while (current_node.parent_idx != -1) {
                shortest_path.path.push_back({get_xy_original(current_node.x, x_min), get_xy_original(current_node.y, y_min)});
                current_node = closed_set.at(current_node.parent_idx);
            }
            std::reverse(shortest_path.path.begin(), shortest_path.path.end());
            break;
        }

        closed_set.insert({get_node_index(current_node), current_node});

        for (auto& direction : directions) {
            AStarNode neighbor = AStarNode(current_node.x + direction.dx, 
                                            current_node.y + direction.dy, 
                                            current_node.g + direction.cost,
                                            get_node_index(current_node));
                                            
            if (closed_set.find(get_node_index(neighbor)) != closed_set.end()) { // in closed set
                continue;
            }
            if (obstacle_map[neighbor.x][neighbor.y]) {
                continue;
            }
            auto it = open_set_map.find(get_node_index(neighbor)); // in open set and has a higer cost
            if (it != open_set_map.end() && it->second.g > neighbor.g) {
                open_set_map.insert({get_node_index(neighbor), neighbor});
            } else if (it == open_set_map.end()) { // not in open set 
                open_set_map.insert({get_node_index(neighbor), neighbor});
                open_set.push(get_node_index(neighbor));
            }
        }
    }

    return shortest_path;
}

int AStar::get_node_index(AStarNode node) {
    return (node.x-x_min) * x_width + (node.y-y_min);
}

int AStar::get_xy_scaled(double xy_original, double xy_min) {
    return round((xy_original-xy_min)/resolution);
}

double AStar::get_xy_original(int xy_scaled, double xy_min) {
    return xy_scaled * resolution + xy_min;
}

// Function to get points along a line using Bresenham's algorithm
std::vector<Point> AStar::get_points_along_line(int x1, int y1, int x2, int y2) {

    std::vector<Point> points;
    
    // Calculate differences and determine the sign of each step
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    
    int error = dx - dy;
    
    int x = x1;
    int y = y1;
    
    while (true) {
        
        points.push_back({x, y});

        if (x == x2 && y == y2) {
            break;
        }
        
        int e2 = 2 * error;
        
        // Move along the x-axis
        if (e2 > -dy) {
            error -= dy;
            x += sx;
        }
        
        // Move along the y-axis
        if (e2 < dx) {
            error += dx;
            y += sy;
        }
    }

    return points;
}
