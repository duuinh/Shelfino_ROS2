#include "prm.hpp"

PRM::PRM(int k_neighbors, std::vector<Obstacle> obstacles, std::vector<GraphNode> borders)
{
    int n_samples = 1000;
    obstacles_ = obstacles;
    double x_max = std::numeric_limits<double>::min();
    double x_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();

    for (auto &border : borders)
    {
        if (border.x < x_min)
        {
            x_min = border.x;
        }
        if (border.x > x_max)
        {
            x_max = border.x;
        }
        if (border.y < y_min)
        {
            y_min = border.y;
        }
        if (border.y > y_max)
        {
            y_max = border.y;
        }
    }

    // sample nodes
    while ( roadmap.nodes.size() < n_samples)
    {
        Point new_point = generate_new_point(x_min, x_max, y_min, y_max);
        if (is_in_free_space(new_point, borders))
        {
            roadmap.nodes.push_back(new_point);
        }
    }

    // construct a roadmap by finding k nearest neighbors for each node

    roadmap.graph.resize(roadmap.nodes.size());

    for (int i = 0; i < roadmap.nodes.size(); ++i)
    {
        connect_to_neighbors(i);
    }
}

void PRM::connect_to_neighbors(int current_node_index)
{
    std::vector<std::pair<double, int>> neighbors;
    for (int j = 0; j < roadmap.nodes.size(); ++j)
    {
        if (current_node_index != j)
        {
            double dist = distance(roadmap.nodes[current_node_index], roadmap.nodes[j]);
            neighbors.emplace_back(dist, j);
        }
    }

    std::sort(neighbors.begin(), neighbors.end()); // sort neighbors by distance

    for (int k = 0; k < std::min(k_neighbors, (int)neighbors.size()); ++k)
    {
        if (is_obstacle_free(roadmap.nodes[current_node_index], roadmap.nodes[neighbors[k].second]))
        {
            // add this neighbor to current node's neighbors list
            roadmap.graph[current_node_index].push_back(neighbors[k].second);

            // also, add current node this this neighbor's neighbors list
            roadmap.graph[neighbors[k].second].push_back(current_node_index);
        }
    }
}

bool PRM::is_in_free_space(Point &new_point, std::vector<GraphNode> borders)
{
    // check for obstacle
    for (Obstacle &obstacle : obstacles_)
    {
        h2d::Circle robot_circle = h2d::Circle(h2d::Point2d(new_point.x, new_point.y), ROBOT_RADIUS + clearance);

        if (obstacle.type == ObstacleType::CYLINDER)
        {
            h2d::Circle obs_circle = h2d::Circle(h2d::Point2d(obstacle.x, obstacle.y));
            if (obs_circle.intersects(robot_circle).size() > 0)
            {
                return false;
            }
        }
        else if (obstacle.type == ObstacleType::BOX)
        {
            h2d::CPolyline obs_box = h2d::CPolyline(std::vector<h2d::Point2d>{
                {obstacle.x - obstacle.dx / 2.0, obstacle.y + obstacle.dy / 2.0},
                {obstacle.x - obstacle.dx / 2.0, obstacle.y - obstacle.dy / 2.0},
                {obstacle.x + obstacle.dx / 2.0, obstacle.y - obstacle.dy / 2.0},
                {obstacle.x + obstacle.dx / 2.0, obstacle.y + obstacle.dy / 2.0}});
            if (obs_box.intersects(robot_circle).size() > 0)
            {
                return false;
            }
        }

        // check if inside the map
        if (!is_inside_map(new_point, ROBOT_RADIUS + clearance, borders))
        {
            return false;
        }
    }
    return true;
}

bool PRM::is_obstacle_free(Point &new_point, Point &neighbor)
{
    for (Obstacle &obstacle : obstacles_)
    {
        double x1 = new_point.x, y1 = new_point.y;
        double x2 = neighbor.x, y2 = neighbor.y;
        double ox = obstacle.x, oy = obstacle.y;

        double m = (y2 - y1) / (x2 - x1);
        double b = y1 - m * x1;

        // check distance from obstacle (point) to the line
        double obs_radius = 0;
        if (obstacle.type == ObstacleType::CYLINDER)
        {
            obs_radius = obstacle.radius;
        }
        else if (obstacle.type == ObstacleType::BOX)
        {
            obs_radius = std::max(obstacle.dx, obstacle.dy); // simply choose the max value
        }

        // check if obstacle is within bounding box of the line segment
        if (ox >= std::min(new_point.x, neighbor.x) && ox <= std::max(new_point.x, neighbor.x) &&
            oy >= std::min(new_point.y, neighbor.y) && oy <= std::max(new_point.y, neighbor.y))
        {
            // check distance from obstacle (point) to the line
            if (ox >= std::min(new_point.x, neighbor.x) && ox <= std::max(new_point.x, neighbor.x) &&
                std::abs(m * ox - oy + b) / std::sqrt(m * m + 1) < obs_radius + ROBOT_RADIUS + clearance)
            {
                return false;
            }
        }
    }
    return true;
}

bool PRM::is_inside_map(Point point, double radius, std::vector<GraphNode> borders)
{
    bool inside = false;

    h2d::CPolyline map_poly;
    std::vector<h2d::Point2d> vertexes;

    for (GraphNode &border : borders)
    {
        vertexes.push_back(h2d::Point2d(border.x, border.y));
    }

    map_poly = h2d::CPolyline(vertexes);

    h2d::Circle robot_circle = h2d::Circle(h2d::Point2d(point.x, point.y), radius);
    if (robot_circle.isInside(map_poly) && robot_circle.intersects(map_poly).size() == 0)
    {
        inside = true;
    }

    return inside;
}

Point PRM::generate_new_point(double x_min, double x_max, double y_min, double y_max)
{
    static std::random_device rd;
    static std::mt19937 engine(rd());
    std::uniform_real_distribution<> randomX(x_min, x_max);
    std::uniform_real_distribution<> randomY(y_min, y_max);

    return {randomX(engine), round(randomY(engine))};
}

int PRM::get_node_index(Point point)
{
    int index = -1;
    for (size_t i = 0; i < roadmap.nodes.size(); i++)
    {
        if (roadmap.nodes[i].x == point.x && roadmap.nodes[i].y == point.y)
        {
            index = i;
            break;
        }
    }

    if (index == -1)
    {
        roadmap.nodes.push_back(point);
        roadmap.graph.resize(roadmap.nodes.size());

        index = roadmap.nodes.size() - 1;
        connect_to_neighbors(index);
    }
    return index;
}

ShortestPath PRM::find_shortest_path(const Point &start_point, const Point &target_point)
{
    /*
     * AStar Pathfinder
     */
    ShortestPath shortest_path;

    AStarNode start_node = AStarNode(start_point.x, start_point.y, get_node_index(start_point));
    AStarNode target_node = AStarNode(target_point.x, target_point.y, get_node_index(target_point));

    std::unordered_map<int, AStarNode> open_set_map, closed_set;

    // node with the lowest F_score is given the highest priority in the priority queue
    auto compare_nodes = [&](const int &lhs, const int &rhs)
    {
        return open_set_map.at(lhs).getF_score(target_point) > open_set_map.at(rhs).getF_score(target_point);
    };
    std::priority_queue<int, std::vector<int>, decltype(compare_nodes)> open_set(compare_nodes);

    open_set.push(start_node.idx);
    open_set_map.insert({start_node.idx, start_node});

    while (!open_set.empty())
    {
        AStarNode current_node = open_set_map.at(open_set.top());
        open_set.pop();
        open_set_map.erase(current_node.idx);

        if (current_node.get_distance(target_point) == 0)
        {
            shortest_path.length = current_node.g;

            // add last node
            shortest_path.path.push_back({current_node.x, current_node.y});
            current_node = closed_set.at(current_node.parent_idx);

            while (true)
            {
                if (current_node.parent_idx == -1) // first node
                {
                    // add first node
                    shortest_path.path.push_back({current_node.x, current_node.y});
                    break;
                }
                else
                {
                    AStarNode parent = closed_set.at(current_node.parent_idx);
                    shortest_path.path.push_back({current_node.x, current_node.y});
                    current_node = parent;
                }
            }
            std::reverse(shortest_path.path.begin(), shortest_path.path.end());
            break;
        }

        closed_set.insert({current_node.idx, current_node});

        std::vector<int> neighbors = roadmap.graph[current_node.idx]; // {dist, idx}
        for (int i = 0; i < neighbors.size(); ++i)
        {
            AStarNode neighbor_node = AStarNode(roadmap.nodes[neighbors[i]].x,
                                                roadmap.nodes[neighbors[i]].y,
                                                neighbors[i],
                                                current_node.idx,
                                                current_node.g + distance(roadmap.nodes[current_node.idx], roadmap.nodes[neighbors[i]]));

            if (closed_set.find(neighbor_node.idx) != closed_set.end())
            { // in closed set
                continue;
            }

            auto it = open_set_map.find(neighbor_node.idx);
            // if in open set and has a higher cost
            if (it != open_set_map.end() && it->second.g > neighbor_node.g)
            {
                open_set_map.insert({neighbor_node.idx, neighbor_node});
            }
            else if (it == open_set_map.end())
            { // not in open set
                open_set_map.insert({neighbor_node.idx, neighbor_node});
                open_set.push(neighbor_node.idx);
            }
        }
    }

    return shortest_path;
}