//om
# pragma once


# include <iostream>
# include <ros/ros.h>
# include <opencv2/highgui/highgui.hpp>
# include <opencv2/imgproc/imgproc.hpp>
# include <vector>
# include <unordered_set>
# include <queue>
# include <string>
# include <math.h>
#define INFINITE 2147483648

class AStar{
    public:

        // Node Structure for A* algorithm
        struct Node{

            int i;              // pixel i
            int j;              // pixel i

            double f;           // total cost
            double g;           // cost from start
            double h;           // cost to goal

            int cost{0};        // cell cost

            bool isClosed{false};
            bool isInOpenList{false};

            std::vector<Node*> neighbours;  // [N, S, W, E, NW, NE, SE, SW]
            Node* parent;

            Node() : f(INFINITE), g(INFINITE), h(0.0), parent(NULL) {}
            Node(int i, int j) : i(i), j(j), f(INFINITE), g(INFINITE), h(0.0), parent(NULL) {}

        };

        // Direction deltas
        enum Direction{
            e_North=0, 
            e_South=1,
            e_West=2, 
            e_East=3,
            e_NorthWest=4,
            e_NorthEast=5,
            e_SouthEast=6,
            e_SouthWest=7
        };


        // compare function for priority queue
        struct compare{
            bool operator()(const Node* a, const Node* b);
        };


        /**
        * \brief      default constructor
        */
        AStar();


        /**
        * \brief      parameterized contructor
        * 
        * \param     grid - accepts the cv::mat grid to plan a path upon
        */
        AStar(cv::Mat grid);


        /**
        * \brief     class destructor
        */
        ~AStar();


        /**
        * \brief      accepts grid from user (incase default c'tor is called)
        * 
        * \param     grid - accepts the cv::mat grid to plan a path upon
        */
        void getGrid(cv::Mat grid);

        /**
        * \brief      receives and sets planning params
        * 
        * \param     gridConnections - 4 connected / 8 connected grid
        * 
        * \param     gridConnections - obstacle threshold value
        */
        void setParams(int gridConnections, int obstacleThreshold);


        /**
        * \brief      sets start and goal pixels
        * 
        * \param     start - start pixel location
        * 
        * \param     goal - goal pixel location
        */
        void setStartAndGoalPixels(std::vector<int> start, std::vector<int> goal);


        /**
        * \brief      initiates the A* algorithm
        */
        std::vector<std::vector<int>> makePlan();
        

    private:


        std::priority_queue<Node*, std::vector<Node*>, compare> _openNodes;

        std::vector<std::vector<Node*>> _nodeGrid;

        // store path in reverse order
        std::vector<Node*> _bestPathInReverse;
        // store final path
        std::vector<std::vector<int>> _path;



        cv::Mat _grid;
        cv::Mat _grid2 = cv::Mat(384, 384, CV_8UC3, cv::Scalar(255,255,255));

        // grid image parameters
        std::string _windowName{"A star window"};
        int _gridRows{100};
        int _gridCols{100};
        int _gridConnections{8};
        int _obstacleThreshold{200};       // considers pixels below 150 intensity in grid as obstacle


        // start and end PIXEL coords
        int _iStart{0}, _jStart{0};
        int _iGoal{0}, _jGoal{0};


        /**
        * \brief      creates node for every pixel value
        */ 
        void _createNodes();


        /**
        * \brief      update heuristic for all nodes 
        */
        void _updateHeuristic();


        /**
        * \brief      populate neighbours for every nodes
        */
        void _populateNeighbours();


        /**
        * \brief      updates i as per direction delta
        * 
        * \param     i - current node's i
        * 
        * \param     dirDelta - [N, S, W, E, NW, NE, SE, SW]
        */
        int _getNewI(int i, Direction dirDelta);


        /**
        * \brief      updates j as per direction delta
        * 
        * \param     j - current node's j
        * 
        * \param     dirDelta - [N, S, W, E, NW, NE, SE, SW]
        */
        int _getNewJ(int j, Direction dirDelta);


        /**
        * \brief      inserts neighbour node into nodegrid if it is valid
        * 
        * \param     i - current node's i
        * 
        * \param     j - current node's j
        * 
        * \param     dirDelta - [N, S, W, E, NW, NE, SE, SW]
        */
        void _validateAndInsertNeighbourNode(int i, int j, Direction dirDelta);


        /**
        * \brief      checks if a node is valid
        * 
        * \param     i - current node's i
        * 
        * \param     j - current node's j
        */
        bool _isValidNode(int i, int j);


        /**
        * \brief      updates cost of all neighbour node's 
        *             of current node and adds them to open list
        * 
        * \param     i - current node's i
        * 
        * \param     j - current node's j
        */
        void _exploreNeighbours(int i, int j);


        /**
        * \brief      backtraces from goal node to start node
        *             and stores them in a vector in reverse
        */
        void _updateBestPathInReverse();


        /**
        * \brief      extracts and stores the path from start 
        *             to goal
        */
        void _updateBestPathInFront();


        /**
        * \brief      computes euclidean dist between two points
        * 
        * \param     i1 - i coord of pixel 1
        * 
        * \param     j1 - j coord of pixel 1
        *        
        * \param     i2 - i coord of pixel 2
        * 
        * \param     j2 - j coord of pixel 2
        */
        double _euclideanDistance(int i1, int j1, int i2, int j2);


        /** 
        * \brief     displays the final grid with path
        */
        void _displayGrid();
};