#include "mr_goto/mr_goto_bfs.hpp"

bool GoToBFS::is_in_grid(tuw::Point2D p, cv::Mat *grid) {
    if (p.x() < 0 || p.x() >= grid->cols) return false;
    if (p.y() < 0 || p.y() >= grid->rows) return false;
    return true;
}

std::vector<tuw::Point2D> GoToBFS::get_neighbours(tuw::Point2D *p, cv::Mat *grid) {
    std::vector<tuw::Point2D> neighbours;
    cv::Point t(p->x(), p->y() - 1);
    cv::Point r(p->x() + 1, p->y());
    cv::Point b(p->x(), p->y() + 1);
    cv::Point l(p->x() - 1, p->y());
    if (p->x() == 0 && p->y() == 0) {
        std::cout << "Value: " << int(grid->at<uchar>(t)) << std::endl;
        std::cout << "Value: " << int(grid->at<uchar>(r)) << std::endl;
        std::cout << "Value: " << int(grid->at<uchar>(b)) << std::endl;
        std::cout << "Value: " << int(grid->at<uchar>(l)) << std::endl;
    }
    if (is_in_grid(t, grid) && grid->at<uchar>(t) == 0) neighbours.emplace_back(t);
    if (is_in_grid(r, grid) && grid->at<uchar>(r) == 0) neighbours.emplace_back(r);
    if (is_in_grid(b, grid) && grid->at<uchar>(b) == 0) neighbours.emplace_back(b);
    if (is_in_grid(l, grid) && grid->at<uchar>(l) == 0) neighbours.emplace_back(l);
    return neighbours;
}

bool GoToBFS::contains(const tuw::Point2D p, const std::vector<tuw::Point2D> v) {
    for (auto &e : v) {
        if (p.x() == e.x() && p.y() == e.y()) return true;
    }
    return false;
}

std::vector<tuw::Point2D> GoToBFS::search(tuw::Point2D start, tuw::Point2D dest, cv::Mat *grid) {
    Node s(start, nullptr);
    std::queue<Node*> q;
    q.push(&s);

    std::vector<tuw::Point2D> visited;
    std::vector<Node*> all_nodes; // all the created pointers will be added to this vector so we can clean them up afterwards
    Node *dest_node = nullptr;
    while (q.size() > 0) {
        Node *n = q.front();
        q.pop();
        visited.emplace_back(n->p);

        if (n->p.x() == dest.x() && n->p.y() == dest.y()) {
            dest_node = n;
            break;
        }

        std::vector<tuw::Point2D> neighbours = get_neighbours(&n->p, grid);
        for (auto neighbour : neighbours) {
            if (!contains(neighbour, visited)) {
                Node *nn = new Node(neighbour, n);
                q.push(nn);
                all_nodes.emplace_back(nn);
            }
        }
    }

    std::vector<tuw::Point2D> waypoints;

    if (dest_node == nullptr) {
        return waypoints;
    } else {
        Node *n = dest_node;
        while (n->prev != nullptr) {
            waypoints.emplace_back(n->p);
            n = n->prev;
        }
    }
    for (auto n : all_nodes) {
        free(n);
    }
    reverse(waypoints.begin(), waypoints.end());
    return waypoints;
}