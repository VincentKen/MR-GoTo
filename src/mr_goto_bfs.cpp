#include "mr_goto/mr_goto_bfs.hpp"
#include "opencv2/opencv.hpp"
#include <tuw_geometry/tuw_geometry.hpp>

using namespace std;
using namespace mr;

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

    if (is_in_grid(t, grid) && grid->at<uchar>(t) == 0) neighbours.emplace_back(t);
    if (is_in_grid(r, grid) && grid->at<uchar>(r) == 0) neighbours.emplace_back(r);
    if (is_in_grid(b, grid) && grid->at<uchar>(b) == 0) neighbours.emplace_back(b);
    if (is_in_grid(l, grid) && grid->at<uchar>(l) == 0) neighbours.emplace_back(l);
    return neighbours;
}

bool GoToBFS::contains(const tuw::Pose2D p, const std::vector<tuw::Pose2D> v) {
    for (auto &e : v) {
        if (p.x() == e.x() && p.y() == e.y()) return true;
    }
    return false;
}

std::vector<tuw::Pose2D> GoToBFS::search(tuw::Pose2D start, tuw::Pose2D dest, cv::Mat *grid) {
    Node s(start, nullptr);
    std::queue<Node*> q;
    q.push(&s);
    std::vector<tuw::Pose2D> visited;
    std::vector<Node*> all_nodes; // all the created pointers will be added to this vector so we can clean them up afterwards
    Node *dest_node = nullptr;
    while (q.size() > 0) {
        cout <<  "BFS Queue size: " << q.size() << "\r";
        fflush ( stdin );
        Node *n = q.front();
        q.pop();
        visited.emplace_back(n->p);

        if (n->p.x() == dest.x() && n->p.y() == dest.y()) {
            dest_node = n;
            break;
        }

        std::vector<tuw::Point2D> neighbours = get_neighbours(&n->p.position(), grid);
        for (auto neighbour : neighbours) {
            if (!contains(neighbour, visited)) {
                Node *nn = new Node(tuw::Pose2D(neighbour, 0), n);
                q.push(nn);
                all_nodes.emplace_back(nn);
            }
        }
    }
    std::cout << std::endl;
    std::vector<tuw::Pose2D> waypoints;

    if (dest_node == nullptr) {
        return waypoints;
    } else {
        Node *n = dest_node;
        while (n->prev != nullptr) {
            double theta = n->p.theta(); // angle the previous node should be in to point to this node
            if (n->prev->p.x() == n->p.x() - 1) { // this node is to the right of the previous
                theta = 0;
            }
            if (n->prev->p.x() == n->p.x() + 1) { // this node is to the left of the previous so the previous should turn 180 degrees
                theta = M_PI;
            }
            if (n->prev->p.y() == n->p.y() - 1) { // this node is to the bottom of the previous
                theta = 3*M_PI/2;
            }
            if (n->prev->p.y() == n->p.y() + 1) { // this node is to the top of the previous
                theta = M_PI/2;
            }
            n->prev->p.set_theta(theta);
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