#include <tuw_geometry/tuw_geometry.hpp>
#include "opencv2/opencv.hpp"
#include <algorithm>

/**
 * Breadth First search classed used to find waypoints towards target 
 */
class GoToBFS {
private:
    struct Node {
        Node(tuw::Point2D p, Node *prev) : p(p), prev(prev) {}
        tuw::Point2D p;
        Node *prev;
    };

    /**
     * Checks if provided point is present in the grid
     */
    static bool is_in_grid(tuw::Point2D p, cv::Mat *grid);

    /**
     * Gets all the neighbouring points of p
     * A neighbour is any node which is present inside the grid and is traversable
     * In this case this is a point in the grid with value == 0 
     */
    static std::vector<tuw::Point2D> get_neighbours(tuw::Point2D *p, cv::Mat *grid);

    /**
     * Traverses the vector to see if it contains p 
     */
    static bool contains(const tuw::Point2D p, const std::vector<tuw::Point2D> v);
public:

    /**
     * Performs a search for a path from @p start to @p dest in the provided grid @p grid
     * @param start Start point from which to search for a path to the destination
     * @param dest  The destination to search for
     * @param grid  Grid to search for path on. Note, only values with 0 are seen as traversable
     */
    static std::vector<tuw::Point2D> search(tuw::Point2D start, tuw::Point2D dest, cv::Mat *grid);
};