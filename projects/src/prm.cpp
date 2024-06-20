#include "prm.hpp"

PRM::PRM(std::vector<Obstacle> obstacles, std::vector<GraphNode> borders, std::vector<GraphNode> victims)
{
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

    for (GraphNode &victim : victims)
    {
        roadmap.nodes.push_back({victim.x, victim.y});
    }

    // sample nodes
    std::vector<Point> random_points = generate_poisson_disk_samples(x_min, x_max, y_min, y_max, spacing, victims);
    for (Point &new_point : random_points) {
        if (is_in_free_space(new_point, borders))
        {
            roadmap.nodes.push_back(new_point);
        }
    }
    
    // construct a roadmap by finding k nearest neighbors for each node

    roadmap.graph.resize(roadmap.nodes.size());

    for (size_t i = 0; i < roadmap.nodes.size(); ++i)
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
        h2d::Circle robot_circle = h2d::Circle(h2d::Point2d(new_point.x, new_point.y), ROBOT_RADIUS + INFLATION_RADIUS);

        if (obstacle.type == ObstacleType::CYLINDER)
        {
            h2d::Circle obs_circle = h2d::Circle(h2d::Point2d(obstacle.x, obstacle.y), obstacle.radius);
            if (robot_circle.isInside(obs_circle) || obs_circle.intersects(robot_circle).size() > 0)
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
                
            if (robot_circle.isInside(obs_box) || obs_box.intersects(robot_circle).size() > 0)
            {
                return false;
            }
        }

        // check if inside the map
        if (!is_inside_map(new_point, ROBOT_RADIUS + INFLATION_RADIUS, borders))
        {
            return false;
        }
    }
    return true;
}

bool PRM::is_obstacle_free(Point &new_point, Point &neighbor)
{
    // check for obstacle
    for (Obstacle &obstacle : obstacles_)
    {
        h2d::Segment path_segment = h2d::Segment(h2d::Point2d(new_point.x, new_point.y), h2d::Point2d(neighbor.x, neighbor.y));

        if (obstacle.type == ObstacleType::CYLINDER)
        {
            h2d::Circle obs_circle = h2d::Circle(h2d::Point2d(obstacle.x, obstacle.y), obstacle.radius + INFLATION_RADIUS);
            if (obs_circle.intersects(path_segment).size() > 0)
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
                
            if (obs_box.intersects(path_segment).size() > 0)
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

std::vector<Point> PRM::generate_poisson_disk_samples(double x_min, double x_max, double y_min, double y_max, double min_dist, std::vector<GraphNode> victims) {
    const double cell_size = min_dist / sqrt(2.0);
    int grid_width = static_cast<int>(ceil((x_max - x_min) / cell_size));
    int grid_height = static_cast<int>(ceil((y_max - y_min) / cell_size));

    // grid to keep track of points
    std::vector<std::vector<Point>> grid(grid_width * grid_height);

    std::queue<Point> active_points;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);

    // func. to check if a point is within bounds and minimum distance constraints
    auto isPointValid = [&](const Point& pt) {
        if (pt.x < x_min || pt.x >= x_max || pt.y < y_min || pt.y >= y_max)
            return false;

        int cell_x = static_cast<int>((pt.x - x_min) / cell_size);
        int cell_y = static_cast<int>((pt.y - y_min) / cell_size);

        int search_start_x = std::max(0, cell_x - 2);
        int search_end_x = std::min(grid_width - 1, cell_x + 2);
        int search_start_y = std::max(0, cell_y - 2);
        int search_end_y = std::min(grid_height - 1, cell_y + 2);

        for (int gx = search_start_x; gx <= search_end_x; ++gx) {
            for (int gy = search_start_y; gy <= search_end_y; ++gy) {
                const auto& cell = grid[gx + gy * grid_width];
                for (const auto& neighbor : cell) {
                    double dist_sq = (pt.x - neighbor.x) * (pt.x - neighbor.x) + (pt.y - neighbor.y) * (pt.y - neighbor.y);
                    if (dist_sq < min_dist * min_dist) {
                        return false;
                    }
                }
            }
        }

        // check distance constraints with victime nodes
        for (GraphNode &victim : victims) {
            double dist_sq = (pt.x - victim.x) * (pt.x - victim.x) + (pt.y - victim.y) * (pt.y - victim.y);
            if (dist_sq < min_dist * min_dist) {
                return false;
            }
        }

        return true;
    };

    // gen first point
    Point first_point;
    first_point.x = uniform_dist(gen) * (x_max - x_min) + x_min;
    first_point.y = uniform_dist(gen) * (y_max - y_min) + y_min;

    active_points.push(first_point);
    grid[static_cast<int>((first_point.x - x_min) / cell_size) + static_cast<int>((first_point.y - y_min) / cell_size) * grid_width].push_back(first_point);

    std::vector<Point> points;
    points.push_back(first_point);

    // gen other points
    while (!active_points.empty()) {
        Point pt = active_points.front();
        active_points.pop();

        for (int i = 0; i < 30; ++i) { // try up to 30 times
            double angle = 2 * M_PI * uniform_dist(gen);
            double radius = min_dist + uniform_dist(gen) * min_dist;

            Point new_pt;
            new_pt.x = pt.x + radius * cos(angle);
            new_pt.y = pt.y + radius * sin(angle);

            if (isPointValid(new_pt)) {
                active_points.push(new_pt);
                points.push_back(new_pt);

                int cell_x = static_cast<int>((new_pt.x - x_min) / cell_size);
                int cell_y = static_cast<int>((new_pt.y - y_min) / cell_size);
                grid[cell_x + cell_y * grid_width].push_back(new_pt);
            }
        }
    }

    return points;
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

            if (current_node.parent_idx != -1) { // not first node
                current_node = closed_set.at(current_node.parent_idx);
            }

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