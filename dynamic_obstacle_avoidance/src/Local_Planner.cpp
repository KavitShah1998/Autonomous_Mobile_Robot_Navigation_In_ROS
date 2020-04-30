#include<dynamic_obstacle_avoidance/Local_Planner.hpp>

local_planner::local_planner(Map_Manager manager, int step_size){
  Cfree_ = manager_.getCfree();
  regional_manager_ = manager;
  branch_length = step_size;
  region_radius = 1.5*step_size;
}

std::vector<geometry_msgs::PoseStamped> local_planner::makePlan(std::vector<int> root, std::vector<int> goal);

local_planner::~local_planner(){

}

float local_planner::calculateDistance_(std::vector<int> first_point, std::vector<int> second_point){
  return sqrt( pow(second_point[0] - first_point[0],2) + pow(second_point[1] - first_point[1],2) );
}

bool local_planner::hasObstacle_(std::vector<int> Xnearest, std::vector<int> Xnew){
  // check if the straight line path between any two points (in particular Xneared and Xnew ) is free from obstacles{false} or not{true}

  int i1,i2,j1,j2,i,j;
  float m;
  float di,dj;
  //step 1: set i1,i2,j1,j2 as per smallest i coordinate
  if(Xnearest[0]< Xnew[0]){
    i1= Xnearest[0];
    j1= Xnearest[1];

    i2= Xnew[0];
    j2= Xnew[1];
  }

  else if(Xnearest[0]> Xnew[0]){
    i1= Xnew[0];
    j1= Xnew[1];

    i2= Xnearest[0];
    j2= Xnearest[1];
  }
  else{  // (Xnearst[0] == Xnew[0])
    if(Xnearest[1]< Xnew[1]){
      i1 = Xnearest[0];
      j1 = Xnearest[0];

      i2= Xnew[0];
      j2= Xnew[1];
    }
    else{
      i1 = Xnew[0];
      j1 = Xnew[1];
      i2= Xnearest[0];
      j2= Xnearest[1];
    }
  }

  // step2: set di, dj as per m;
  dj= (j2-j1);
  di= (i2-i1);
  m= dj/di;

  if(abs(m)<=1){
    di=1;
    dj=m*di;
  }
  else if(abs(m)>1 && di!=0){
    dj=1;
    di = dj/m;
  }
  else{ // di=0
    di = 0;
    dj=1;
  }


  i=i1;
  j=j1;
  // step3: the while loop
  while(i!=i2 && j!=j2){
    if(manager.checkObstacle(std::vector<int> vec{i,j})==true){
      return true;
    }
    i+=di;
    j+=dj;
  }

  // if execution completes the while loop and reaches till here, it means no obstacle, Therefore return false;
  return false;

}

std::vector<int> local_planner::newNode_(std::vector<int> Xnear, std::vector<int> Xrand){
  int i,j,delta_i_,delta_j_;
  double m,di,dj;
  std::vector<int> point_1, point_2;

  delta_i_ = Xrand[0]-Xnear[0];
  delta_j_ = Xrand[1]-Xnear[1];

  if(delta_i_ != 0){
    m = delta_j_/delta_i_;
    di = branch_length/sqrt(1+pow(m,2));
    dj = m*di;

    point_1.emplace_back((int)(Xnear[0] + di));
    point_1.emplace_back((int)(Xnear[1] + dj));

    point_2.emplace_back((int)(Xnear[0]-di));
    point_2.emplace_back((int)(Xnear[0]-dj));
  }
  else{  // delta_i_ = 0
        di = 0;
        dj = branch_length;
        point_1.emplace_back((int)(Xnear[0]));
        point_1.emplace_back((int)(Xnear[1] + dj));
        point_2.emplace_back((int)(Xnear[0]));
        point_2.emplace_back((int)(Xnear[1] - dj));
  }

/*  if(calculateDistance_(point_1,Xrand) < calculateDistance_(point_2,Xrand)){
    return point_1;
  }
  else{
    return point_2;
  }
  */
  return ((calculateDistance_(point_1,Xrand)< calculateDistance_(point_2,Xrand))? point_1:point_2);
}

void local_planner::reversePath_(){
  for(int ii = path_.size()-1; ii>=0;ii--){
    reversePath_[path_.size()-1-ii] = path_[ii];
  }
}
