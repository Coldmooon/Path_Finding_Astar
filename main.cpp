#include <iostream>
#include <queue>
#include <vector>
#include <opencv2/opencv.hpp>

// A node in a graph is indexed by 'idx'. Take 'cost' to go through it.
struct Node {
    int idx;
    float cost;
    Node(int x, float y): idx(x), cost(y) {}
};

struct compare_cost {
    bool operator()(Node x, Node y) {
        return x.cost > y.cost;
    }
};

// return L1 distance between two nodes.
float heuristic (int sx, int sy, int ex, int ey) {
    return std::abs(sx - ex) + std::abs(sy - ey);
}

void astar(unsigned char * map, int width, int height, int s, int e, int * came_from) {
    std::priority_queue<Node, std::vector<Node>, compare_cost> frontier;
    frontier.push(Node(s, 0));

    // Each node in the graph uses 'cost_so_far' to measure the cost of walking from the start point to it.
    // Initialize 'cost_so_far' with INF means that a node has not been visited or it is a wall.
    const float INF = std::numeric_limits<float>::infinity();
    float * cost_so_far = new float[width * height];
    for (int i = 0; i < width * height; ++i)
        cost_so_far[i] = INF;
    cost_so_far[s] = 0;
    came_from[s] = -1;

    while ( !frontier.empty() ) {
        Node current = frontier.top();
        if (current.idx == e)
            break;

        frontier.pop();

        // the neighbors of the current node: top, down, left, and right.
        int neighbors[4];
        neighbors[0] = current.idx / width > 0 ? current.idx - width : -1;
        neighbors[1] = current.idx % width > 0 ? current.idx - 1: -1;
        neighbors[2] = current.idx / width + 1 < height ? current.idx + width : -1;
        neighbors[3] = (current.idx + 1) % width > 0 ? current.idx + 1 : -1;

        // for each neighbors, new_cost measures the cost of walking from the start point to it, and heuristic measures
        // how far it is from it to the end point.
        for (int i = 0; i < 4; ++i) {
            float moving_cost = map[neighbors[i]] == '\0' ? INF : 1;
            float new_cost = cost_so_far[current.idx] + moving_cost;
            if (new_cost < cost_so_far[neighbors[i]]) {
                cost_so_far[neighbors[i]] = new_cost;
                float priority = new_cost + heuristic(neighbors[i] % width, neighbors[i] / width, e % width, e / width);
                frontier.push(Node(neighbors[i], priority));
                came_from[neighbors[i]] = current.idx;
            }
        }
    }
    delete [] cost_so_far;
}

int main(int argc, const char * argv[]) {
    std::string maze_large = "mazes/maze_large.png";
    std::string maze_small = "mazes/maze_small.png";

    cv::Mat image, image_gray;
    image = cv::imread(maze_large);
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    int width = image_gray.cols;
    int height = image_gray.rows;
    int dims = width * height;

    // turn the 2-D graph to a 1-D array for convenience.
    unsigned char * map = new unsigned char[dims];
    if (image_gray.isContinuous())
        map = image_gray.data;

    // set the start point and the end point.
    // start point: the first while pixel in the first row.
    // end   point: the first while pixel in the last col.
    // black pixel denotes a wall.
    int start_point = 0;
    int end_point = 0;
    uchar * sp = image_gray.ptr<uchar>(start_point);
    while ((int)*sp != 255) {
        start_point++;
        sp++;
    }
    uchar * ep = image_gray.ptr<uchar>(end_point);
    while ( (int)ep[width - 1] != 255) {
        end_point++;
        ep = image_gray.ptr<uchar>(end_point);
    }
    end_point = end_point * width + width - 1;

    // came_from: record the path 
    int * came_from = new int[dims];
    astar(map, width, height, start_point, end_point, came_from);

    // draw the path.
    int draw = end_point;
    while (draw != start_point) {
        int x = draw % width;
        int y = draw / width;

        uchar * coloring = image.ptr<uchar>(y);
        coloring[x * 3] = 64; // blue
        coloring[x * 3 + 1] = 61; // green
        coloring[x * 3 + 2] = 214; // red

        draw = came_from[draw];
    }

    int startx = start_point % width;
    int starty = start_point / width;
    uchar * start_coloring = image.ptr<uchar>(starty);
    start_coloring[startx * 3] = 0;
    start_coloring[startx * 3 + 1] = 255;
    start_coloring[startx * 3 + 2] = 0;

    cv::imshow("path", image);
    cv::waitKey(0);
    // cv::imwrite("path.jpg", image);

    delete [] came_from;
    return 0;
}
