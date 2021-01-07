// om

# include "hierarchical_motion_planner/a_star.h"

/* Default constructor */
AStar :: AStar(){

}


/* Parameterized contructor */
AStar :: AStar(cv::Mat grid) : _grid(grid){


    // works on 1 channel monochrome image only
    if(_grid.channels() == 3)
        cv::cvtColor(_grid, _grid, CV_BGR2GRAY);


    // setup params
    _gridRows = _grid.rows;
    _gridCols = _grid.cols;


    // creating a color image for final path animation (Not Necessary)
    for(int i=0; i<_gridRows; i++){
        for(int j=0; j<_gridCols; j++){

            _grid2.at<cv::Vec3b>(i,j) = cv::Vec3b(_grid.at<uchar>(i,j), _grid.at<uchar>(i,j), _grid.at<uchar>(i,j));
        }
    }


    // reserving memory for the 2D vector of nodes
    _nodeGrid.resize(_gridRows, std::vector<Node*>(_gridCols));


    // std::cout << "Image channels : " << _grid.channels() << "\n";

    // create nodes for each pixel
    _createNodes();

    
}


/* Class destructor */
AStar :: ~AStar(){

}


/* Cccepts grid from user (incase default c'tor is called) */
void AStar :: getGrid(cv::Mat grid){
    
    
    _grid = grid;

// works on 1 channel monochrome image only
    if(_grid.channels() == 3)
        cv::cvtColor(_grid, _grid, CV_BGR2GRAY);


    // setup params
    _gridRows = _grid.rows;
    _gridCols = _grid.cols;


    // creating a color image for final path animation (Not Necessary)
    for(int i=0; i<_gridRows; i++){
        for(int j=0; j<_gridCols; j++){

            _grid2.at<cv::Vec3b>(i,j) = cv::Vec3b(_grid.at<uchar>(i,j), _grid.at<uchar>(i,j), _grid.at<uchar>(i,j));
        }
    }


    // reserving memory for the 2D vector of nodes
    _nodeGrid.resize(_gridRows, std::vector<Node*>(_gridCols));


    // std::cout << "Image channels : " << _grid.channels() << "\n";

    // create nodes for each pixel
    _createNodes();


}


/* comparator function for priority queue */
bool AStar :: compare :: operator() (const Node* a, const Node* b) {
    return a->f > b->f;
}


/* Sets start and goal pixels */
void AStar :: setStartAndGoalPixels(std::vector<int> start, std::vector<int> goal){

    // update the start and goal points
    _iStart = start[0];
    _jStart = start[1];

    _iGoal = goal[0];
    _jGoal = goal[1];


    // once the obtained the goal points, update the hueristic to the goal for every node
    _updateHeuristic();


    // initialize the start node 
    _nodeGrid[_iStart][_jStart]->g = 0;
    _nodeGrid[_iStart][_jStart]->f = _nodeGrid[_iStart][_jStart]->h;

}


/* Receives and sets planning params */
void AStar :: setParams(int gridConnections=4, int obstacleThreshold=200){

    _gridConnections = gridConnections;
    _obstacleThreshold = obstacleThreshold;


    // populate the neighbours for each node based on 4-connected or 8-connected
    _populateNeighbours();
}


/* Creates node for every pixel value */
void AStar :: _createNodes(){
    
    for(int i=0; i<_gridRows; i++){
        for(int j=0; j<_gridCols; j++){

            _nodeGrid[i][j] = new Node(i,j);
            _nodeGrid[i][j]->cost = _grid.at<uchar>(i,j);
        }
    }

}


/* Populate neighbours for every nodes */
void AStar :: _populateNeighbours(){

    // check and populate the valid neighbours
    for(int i=0; i<_gridRows; i++){

        for(int j=0; j<_gridCols; j++){

            // the system can either be 4 connected or 8 connected
            bool b_is8Connected = (_gridConnections == 8) ? true : false;


            // Neighbour Order  :  N  ,  S  ,  W  ,  E  ,  NW  ,  NE  ,  SE  ,  SW  

            // 4 basic directions
            _validateAndInsertNeighbourNode(i,j, e_North);

            _validateAndInsertNeighbourNode(i,j, e_South);

            _validateAndInsertNeighbourNode(i,j, e_West);

            _validateAndInsertNeighbourNode(i,j, e_East);

            if(b_is8Connected){
                // other  4 directions
                _validateAndInsertNeighbourNode(i,j, e_NorthWest);

                _validateAndInsertNeighbourNode(i,j, e_NorthEast);

                _validateAndInsertNeighbourNode(i,j, e_SouthEast);

                _validateAndInsertNeighbourNode(i,j, e_SouthWest);
            }

        }

    }

}


/* Inserts neighbour node into nodegrid if it is valid */
void AStar :: _validateAndInsertNeighbourNode(int i, int j, Direction directionDelta){

    // get i & j of neighbour node
    int newI = _getNewI(i, directionDelta);
    int newJ = _getNewJ(j, directionDelta);

    // check and insert the neighbour node
    if(_isValidNode(newI, newJ)){

        ((_nodeGrid[i][j])->neighbours).push_back(_nodeGrid[newI][newJ]);
    }
    else{ // insert NULL otherwise
        
        ((_nodeGrid[i][j])->neighbours).push_back(NULL);
    }
}


/* Updates i as per direction delta */
int AStar :: _getNewI(int i, Direction dirDelta){

    int iIncrement[8] = {-1, +1, 0, 0, -1, -1, +1, +1};

    return i + iIncrement[dirDelta];
}


/* Updates j as per direction delta */
int AStar :: _getNewJ(int j, Direction dirDelta){

    int jIncrement[8] = {0, 0, -1, +1, -1, +1, +1, -1};

    return j + jIncrement[dirDelta];
}


/* checks if a node is valid */
bool AStar :: _isValidNode(int i, int j){

    if( (i>=0 && i<_gridRows) && (j>=0 && j<_gridCols) && _nodeGrid[i][j]->cost < _obstacleThreshold)
        return true;

    return false;
}


/* Updates heuristic for every node */
void AStar :: _updateHeuristic(){

    for(int i=0; i<_gridRows; i++){
        for(int j=0; j<_gridCols; j++){

            _nodeGrid[i][j]->h = _euclideanDistance(i,j,_iGoal, _jGoal);
        }
    }

    
}


/* Computes euclidean dist between two points */
double AStar :: _euclideanDistance(int i1, int j1, int i2, int j2){
    return sqrt( pow(i2-i1,2) + pow(j2-j1,2) );
}



/* Updates cost of all neighbour node's of current node and adds them to open list */
void AStar :: _exploreNeighbours(int i, int j){


    Node* currentNode = _nodeGrid[i][j];

    for(int k=0; k<_gridConnections ; k++){


        Node* neighbourNode = currentNode->neighbours[k];

        // if neighbour is not null
        if(neighbourNode){

            // neighbour not yet explored
            if( !neighbourNode->isInOpenList && !neighbourNode->isClosed ){


                // update cost to come as per grid connections
                if(k < 4)  // 4 - connected
                    neighbourNode->g = currentNode->g + 1;      // cost to go 1 step along {N,S,W,E} is 1 unit

                else       // 8 - connected
                    neighbourNode->g = currentNode->g + 1.41;   // cost to go 1 step along {NW,NE,SE,SW} is sqroot(2) units



                // update total cost of node .. sums up cost-to-come, cost-to-goal, & cell-cost(new feature)
                neighbourNode->f = neighbourNode->g + neighbourNode->h + neighbourNode->cost*100/255;
                
                // set the current node as parent of the neighbour node
                neighbourNode->parent = currentNode;

                // add node to open list
                _openNodes.push(neighbourNode);
                neighbourNode->isInOpenList = true;
            }
        }
    }
}


/* A* algorithm */
std::vector<std::vector<int>> AStar :: makePlan(){

    // add start node to open list
    _openNodes.push(_nodeGrid[_iStart][_jStart]);
    _nodeGrid[_iStart][_jStart]->isInOpenList = true;


    // iterate till open list not empty
    while(!_openNodes.empty()) {

        // get top node
        Node* bestNodeInTheFringe = _openNodes.top();
        bestNodeInTheFringe->isClosed = true;
        _openNodes.pop();
        
        // top node == goal node ?
        if(bestNodeInTheFringe->i == _iGoal && bestNodeInTheFringe->j == _jGoal){
            std::cout << "Global Path Found \n";
            break;
        }
        // explore and add neighbors of top node in open list
        else{
            _exploreNeighbours(bestNodeInTheFringe->i, bestNodeInTheFringe->j);
        }

    }

    // traverse and store the best path from goal to start (as each node stores its parent.. easy back-tracing)
    _updateBestPathInReverse();

    return _path;
}

/* Traverse and store the best path from goal to start (as each node stores its parent.. easy back-tracing) */
void AStar :: _updateBestPathInReverse(){

    Node* currentNode = _nodeGrid[_iGoal][_jGoal];

    while(currentNode->parent!=NULL){
        _bestPathInReverse.emplace_back(currentNode);
        currentNode = currentNode->parent;
    }

    ROS_INFO_STREAM( "bestPathSize = " << _bestPathInReverse.size() );
    _updateBestPathInFront();
    _displayGrid();
}


/* extracts and stores the path from start to goal */
void AStar::_updateBestPathInFront(){

    int size = _bestPathInReverse.size();

    _path.clear();
    _path.reserve(size+1);
    


    for(int i=size-1; i>=0; i--){

        // for animation
        _grid.at<uchar>(_bestPathInReverse[i]->i,_bestPathInReverse[i]->j) = 255;
        _grid2.at<cv::Vec3b>(_bestPathInReverse[i]->i,_bestPathInReverse[i]->j) = cv::Vec3b(0,0,255);

        std::vector<int> point{_bestPathInReverse[i]->i,_bestPathInReverse[i]->j};
        _path.emplace_back( point );
    }

}


/* Displays the 2D grid */
void AStar :: _displayGrid() {

    std::cout << "Image rows : " << _grid.rows << "\n";


    // error if image is empty
    if(!_grid.data){
        ROS_ERROR("Could not load the map image");
        return;
    }


    // display image
    cv::namedWindow(_windowName , cv::WINDOW_AUTOSIZE);
    cv::imshow(_windowName, _grid2);
    auto k = cv::waitKey(1000);

    // destroy image windows
    // if(k==27){
        cv::destroyAllWindows();
    //     exit(0);
    // }
}



/*

    API:

    constructor + image // contructor + getGrid
    setParams
    setStartAndGoalPixels
    makePlan
*/


