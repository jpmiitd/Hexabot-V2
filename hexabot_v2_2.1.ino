#define S_RXD 18
#define S_TXD 19
// #include <esp_task_wdt.h>
#include <SCServo.h>
#include <algorithm>
#include <math.h>
SCSCL sc;
UBaseType_t uxHighWaterMark;
const float t(120.0), m(102.5), _2tm(24600.0), t2(14400.0), m2(10506.25), _2m(205.0), z_comp(26.2), y_comp(52.2), pi(3.14159265);
const int endpoint_low[6][4] = { { 250, 110, 15, 900 },
                                 { 230, 105, 60, 880 },
                                 { 240, 85, 70, 890 },
                                 { 215, 45, 30, 930 },
                                 { 240, 70, 50, 910 },
                                 { 275, 85, 60, 890 } };
const int endpoint_high[6][4] = { { 900, 970, 875, 40 },
                                  { 880, 965, 920, 20 },
                                  { 890, 945, 935, 30 },
                                  { 865, 905, 890, 70 },
                                  { 890, 930, 910, 50 },
                                  { 925, 945, 920, 30 } };

const float off_set[6] = { -60., 0.0, 60.0, 120.0, 180.0, -120.0 };
float bz_pts[3][11][3];
float lin_pts[3][11][3];
int loc(0), max_trajec_pts, move, max_disp;
float last_ang, curr_ang, gen_speed=400;
unsigned long ti, tf;
bool leg_state[6] = {true,true,true,true,true,true};
struct coordinate{
  float x,y,z;
  coordinate()
  {  }
  coordinate(float _x, float _y , float _z)
  {
    x = _x;
    y = _y;
    z = _z;
  }
};
//leg no 5-> separate func 
class LEG {
public:
  u16 pos[4], speed[4]={500,500,500,500}, time[4]={0,0,0,0};
  u8 id,s_id[4];
  float x, y, z;
  LEG(u8 ID) 
  {
    id = ID;
    for (u8 i = 0; i < 4; i++) 
    {
      s_id[i] = (4 * id) + i + 1;
      pos[i] = sc.ReadPos(s_id[i]);
      // Serial.print(speed[i]);
      delay(10);
    }
  }

  void joint(int S_ID_idx, int POS) 
  {
    pos[S_ID_idx] = POS;
    Serial.println("reached till joint");
    Serial.println(speed[S_ID_idx]);
    sc.WritePos(s_id[S_ID_idx], POS, 0, speed[S_ID_idx]);
    Serial.println("reached till joint done");
    Serial.print(s_id[S_ID_idx]);
    Serial.println();
  }
  void joint(int S_ID_idx, int POS, int speed) 
  {
    pos[S_ID_idx] = POS;
    sc.WritePos(s_id[S_ID_idx], POS, 0, speed);
  }
  void syncWrite()
  {
    sc.SyncWritePos(s_id, 4, pos, time, speed);
  }
  void go_to(float X, float Y, float Z);

  int load(int i)  //-1<i<4
  {
    return sc.ReadLoad(s_id[i]);
  }
};

LEG* set[6];

// void print() {
//   for (int i = 0; i <= 0; i++) {
//     for (int j = 1; j <= 4; j++) {
//       Serial.print(" Servo ID : ");
//       Serial.print(i + j);
//       Serial.print(" Pos : ");
//       Serial.print(pos[i][i + j]);
//       Serial.print("\n");
//     }
//   }
// }

float todeg(float rad) {
  return (rad / pi) * 180.0;
}

void print_ang(int leg, float lambda, float theta,float beta,float gama,float delta) {
  Serial.print(leg);
  Serial.print(" Theta : ");
  Serial.print(todeg(theta));
  Serial.print(" Lambda : ");
  Serial.print(todeg(lambda));
  Serial.print(" Beta : ");
  Serial.print(todeg(beta));
  Serial.print(" Gamma : ");
  Serial.print(todeg(gama));
  Serial.print(" Delta : ");
  Serial.print(todeg(delta));
  Serial.print(set[leg]->speed[0]);
  Serial.print(" ");
  Serial.print(set[leg]->speed[1]);
  Serial.print(" ");
  Serial.print(set[leg]->speed[2]);
  Serial.print(" ");
  Serial.print(set[leg]->speed[3]);
  Serial.print(" ");
  Serial.print(" Time taken (us) : ");
  Serial.print(tf - ti);
  Serial.print("\n");
}

bool maximum(float a, float b)
{
  return a>b;
}

void set_speed(int pos0, int pos1, int pos2, int pos3, int leg) //data can be passed as pointer
{
  LEG* l=set[leg];
  Serial.print(pos1);
  Serial.print("\n");
  float del0,del1,del2,del3;
  del0 = float(abs(l->pos[0] - pos0));
  del1 = float(abs(l->pos[1] - pos1));
  del2 = float(abs(l->pos[2] - pos2));
  del3 = float(abs(l->pos[3] - pos3));
  float max_del = max({del0,del1,del2,del3},maximum);
  max_disp = max_del;
  l->speed[0] = int(gen_speed*(del0/max_del));
  // Serial.print()
  l->speed[1] = int(gen_speed*(del1/max_del));
  l->speed[2] = int(gen_speed*(del2/max_del));
  l->speed[3] = int(gen_speed*(del3/max_del));
}

//first x,y,z will be written in the respective objects.... 

void ik(int leg) {
  float lambda, theta, beta, gama, delta;
  ti = micros();
  float temp1 = (sqrt((set[leg]->x * set[leg]->x) + (set[leg]->y * set[leg]->y)) - y_comp);
  float temp2 = (z_comp - set[leg]->z);
  float r = sqrt(temp2 * temp2 + temp1 * temp1);
  if (r >= 222.5) return;
  float r2 = r * r;
  //calculating theta
  theta = asin((z_comp - set[leg]->z) / r);

  //calculating lambda
  if (set[leg]->x)  //for non zero values of x
  {
    lambda = atan(set[leg]->y / set[leg]->x);
    if (lambda < 0) {
      lambda = pi + lambda;
    }
  } else {
    lambda = pi / 2.0;
  }
  // Serial.print(lambda);
  // Serial.print(" ");
  if (lambda < 0 || lambda > pi) return;  //this is to prevent any bizzare movement which may result in something breaking.

  //calculating beta
  beta = acos((t2 + m2 - r2) / _2tm);
  // Serial.print(beta);
  // Serial.print(" ");
  if (beta < 0.5890485 || beta > pi) return;  //this is to prevent any bizzare movement which may result in something breaking.

  //calculating gamma
  gama = acos((m2 + r2 - t2) / (_2m * r));

  //calculating delta
  delta = (pi / 2.0) + theta - gama;
  // Serial.print(delta);
  // Serial.print(" ");
  if (delta < 0 || delta > pi) return;  //this is to prevent any bizzare movement which may result in something breaking.
  tf = micros();
  int pos0 = map(todeg(beta), 33.75, 180, endpoint_low[leg][0], endpoint_high[leg][0]);
  int pos1 = map(90, 0, 180, endpoint_low[leg][1], endpoint_high[leg][1]);
  int pos2 = map(todeg(delta), 0, 180, endpoint_low[leg][2], endpoint_high[leg][2]);
  int pos3 = map(todeg(lambda), 0, 180, endpoint_low[leg][3], endpoint_high[leg][3]);
  //sc.WritePos(id[leg][0], pos[leg][0], 0, 100);
  set_speed(pos0,pos1,pos2,pos3,leg);
  set[leg]->joint(0,pos0);
  Serial.println("reached till leg 1");
  //delay(10);
  // set[leg]->joint(1,pos1);
  // //delay(10);
  // //sc.WritePos(id[leg][2], pos[leg][2], 0, 100);
  // set[leg]->joint(2,pos2);
  // //delay(10);
  // //sc.WritePos(id[leg][3], pos[leg][3], 0, 100);
  // set[leg]->joint(3,pos3);
  //delay(10);
  set[leg]->pos[0] = pos0;
  set[leg]->pos[1] = (endpoint_low[leg][1]+endpoint_high[leg][1])/2;
  set[leg]->pos[2] = pos2;
  set[leg]->pos[3] = pos3;
  set[leg]->syncWrite();

  print_ang(leg, lambda, theta, beta, gama, delta);
}
float toRadian(float deg){
  return (deg/180.)*pi;
}
void LEG::go_to(float X, float Y, float Z)
  {
    x=X;
    y=Y;
    z=Z;
    ik(id);
  }

void rotate_points(float deg,coordinate* start_pt, coordinate* p1_pt, coordinate* p2_pt, coordinate* end_pt){
  float cos_theta=cos(toRadian(deg));
  float sin_theta=sin(toRadian(deg));
  float x_x;
  x_x=start_pt->x;
  start_pt->x=start_pt->x*cos_theta-start_pt->y*sin_theta;
  start_pt->y=x_x*sin_theta+start_pt->y*cos_theta;
  x_x=p1_pt->x;
  p1_pt->x=p1_pt->x*cos_theta-p1_pt->y*sin_theta;
  p1_pt->y=x_x*sin_theta+p1_pt->y*cos_theta;
  x_x=p2_pt->x;
  p2_pt->x=p2_pt->x*cos_theta-p2_pt->y*sin_theta;
  p2_pt->y=x_x*sin_theta+p2_pt->y*cos_theta;
  x_x=end_pt->x;
  end_pt->x=end_pt->x*cos_theta-end_pt->y*sin_theta;
  end_pt->y=x_x*sin_theta+end_pt->y*cos_theta;
  return;
}

// void change_state(u8* ids[], bool* new_state){
//   int i = 0;
//   while(i<ids.size()){
//     state[i] = new_state[i];
//     i++;
//   }
// }
void generate_trajec(float deg, float height) {
  coordinate start_pt,end_pt,p1_pt,p2_pt;
  float m_slope = tan((deg / 180.) * pi);
  float m_slope_2 = m_slope * m_slope;
  float m_disp = ((100 * (1 + m_slope_2)) / (1 + (16 * m_slope_2)));
  float f_slope = tan((deg + 60. / 180.) * pi);
  float f_slope_2 = f_slope * f_slope;
  float f_disp = ((100 * (1 + f_slope_2)) / (1 + (16 * f_slope_2)));
  float b_slope = tan((deg - 60. / 180.) * pi);
  float b_slope_2 = b_slope * b_slope;
  float b_disp = ((100 * (1 + b_slope_2)) / (1 + (16 * b_slope_2)));
  float disp = min({ m_disp, f_disp, b_disp });
  start_pt.x = disp*cos(toRadian(deg));
  start_pt.y = disp*sin(toRadian(deg))+180.;
  start_pt.z = -height;

  end_pt.x = -start_pt.x;
  end_pt.y = -start_pt.y+180.;
  end_pt.z = -height;

  p1_pt.x = start_pt.x/2;
  p1_pt.y = start_pt.y/2;
  p1_pt.z = height/2;
  
  p2_pt.x = end_pt.x/2;
  p2_pt.y = end_pt.y/2;
  p2_pt.z = height/2;

  // for(int i=0;i<3;i++)
  // {
  //     bz_limit_pt[i][0].x = disp*cos(deg+((1-i)*60));
  //     bz_limit_pt[i][0].y = disp*sin(deg+((1-i)*60));
  //     bz_limit_pt[i][0].z = height;
  //     bz_limit_pt[i][1].x = bz_limit_pt[i][0].x/2;
  //     bz_limit_pt[i][1].y = bz_limit_pt[i][0].y/2;
  //     bz_limit_pt[i][1].z = -height/2;
  //     bz_limit_pt[i][2].x = -bz_limit_pt[i][0].x/2;
  //     bz_limit_pt[i][2].y = -bz_limit_pt[i][0].y/2;
  //     bz_limit_pt[i][2].z = -height/2;
  //     bz_limit_pt[i][3].x = - bz_limit_pt[i][0].x;
  //     bz_limit_pt[i][3].y = - bz_limit_pt[i][0].y;
  //     bz_limit_pt[i][3].z = height;
  //     gen_bz_pts(bz_limit_pt[i][0],bz_limit_pt[i][0],bz_limit_pt[i][0],bz_limit_pt[i][0],10,i);
  // }
  Serial.print("disp_generated: ");
  Serial.println(disp);
  gen_bz_pts(start_pt, p1_pt,p2_pt,end_pt,11, 1);
  Serial.println("Leg 1 done");
  rotate_points(120,&start_pt,&p1_pt,&p2_pt,&end_pt);
  Serial.println("Rotation for 2 done");
  gen_bz_pts(start_pt, p1_pt,p2_pt,end_pt,11, 2);
  Serial.println("Leg 2 done");
  rotate_points(180,&start_pt,&p1_pt,&p2_pt,&end_pt);
  Serial.println("Rotation for 0 done");
  gen_bz_pts(start_pt, p1_pt,p2_pt,end_pt,11, 0);
  Serial.println("Leg 0 done");
  for(int k = 0; k<3;k++)
  {
    for(int j = 0; j<3 ; j++)
    {
      for(int i = 0; i<11; i++)
      {
        Serial.print(bz_pts[k][i][j]);
        Serial.print(" ");
      }
      Serial.print("\n");
    }
    Serial.println("");
  }
  int iteration=0;
  Serial.println("Callling boot seq:");
  boot_seq();
  while(iteration <10){
    iteration++;
    Serial.print("Iteration :");
    Serial.println(iteration);
    for(int i = 0; i<sizeof(bz_pts[0])/4; i++){
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print("Stack Space: ");
      Serial.println(uxHighWaterMark);
      Serial.println(i);
      set[0]->go_to(bz_pts[0][i][0], bz_pts[0][i][1], bz_pts[0][i][2]);
      // set[0]->go_to(0.0,180.0,-100.0);
      Serial.println(i);
      set[4]->go_to(bz_pts[1][i][0], bz_pts[1][i][1], bz_pts[1][i][2]);
      Serial.println(i);
      set[2]->go_to(bz_pts[2][i][0], bz_pts[2][i][1], bz_pts[2][i][2]);
      Serial.println(i+1);
      set[3]->go_to(lin_pts[0][10-i][0], lin_pts[0][10-i][1], lin_pts[0][10-i][2]);
      set[1]->go_to(lin_pts[1][10-i][0], lin_pts[1][10-i][1], lin_pts[1][10-i][2]);
      set[5]->go_to(lin_pts[2][10-i][0], lin_pts[2][10-i][1], lin_pts[2][10-i][2]);
      boot_seq();
      leg_state[0] = false;
      leg_state[0] = false;
      
    }
    for(int i = sizeof(bz_pts)/4-1; i>=0; i--){
      set[3]->go_to(bz_pts[0][i][0], bz_pts[0][i][1], bz_pts[0][i][2]);
      set[1]->go_to(bz_pts[1][i][0], bz_pts[1][i][1], bz_pts[1][i][2]);
      set[5]->go_to(bz_pts[2][i][0], bz_pts[2][i][1], bz_pts[2][i][2]);

      set[0]->go_to(lin_pts[0][10-i][0], lin_pts[0][10-i][1], lin_pts[0][10-i][2]);
      set[4]->go_to(lin_pts[1][10-i][0], lin_pts[1][10-i][1], lin_pts[1][10-i][2]);
      set[2]->go_to(lin_pts[2][10-i][0], lin_pts[2][10-i][1], lin_pts[2][10-i][2]);
    }
  }  
}

void crawl(int deg, float height){
  generate_trajec(deg, height);
}

void boot_seq() {
  for (int i = 0; i < 6; i++) {
    Serial.println(i);
    set[i]->go_to(0.0, 180.0,-100.0);
    Serial.println("hi");
  }
}

void gen_bz_pts(coordinate start_pt, coordinate p1_pt, coordinate p2_pt, coordinate end_pt, int num_points, int leg_id)
{
  float deltax = float(start_pt.x-end_pt.x)/(num_points+1), lin_x = end_pt.x;
  float deltay = float(end_pt.y-start_pt.y)/(num_points+1),lin_y = end_pt.y;
  float t_normalized,y,x,z;
  for (int t = 0; t < num_points+1 ; ++t) {
        t_normalized = static_cast<float>(t) / num_points;
        y = start_pt.y + deltay*t;
        x = (pow(1 - t_normalized, 3) * start_pt.x)
                   + (3 * pow(1 - t_normalized, 2) * t_normalized * p2_pt.x)
                   + (3 * (1 - t_normalized) * pow(t_normalized, 2) *p2_pt.x)
                   + pow(t_normalized, 3) * end_pt.x;
        z = (pow(1 - t_normalized, 3) * start_pt.z)
                   + (3 * pow(1 - t_normalized, 2) * t_normalized * p1_pt.z)
                   + (3 * (1 - t_normalized) * pow(t_normalized, 2) * p2_pt.z)
                   + pow(t_normalized, 3) * end_pt.z;
        bz_pts[leg_id][t-1][0] = x;
        bz_pts[leg_id][t-1][1] = y;
        bz_pts[leg_id][t-1][2] = z;
        lin_x+=deltax;
        lin_y-=deltay;
        lin_pts[leg_id][t-1][0] = lin_x;
        lin_pts[leg_id][t-1][1] = lin_y;
        lin_pts[leg_id][t-1][2] = start_pt.z;
    }
    return;
}

void setup() {
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Serial.begin(115200);
  sc.pSerial = &Serial1;
  delay(1000);
  for (int i = 0; i < 6; i++) {
    LEG* place_holder = new LEG(i);
    set[i] = place_holder;
    Serial.println(set[i]->id);
    //set[i]->joint(1,int(endpoint_low[i][1] + 430));
    delay(10);
  }
  boot_seq();
  Serial.print("Stack space: ");
  Serial.println(uxHighWaterMark);
  // esp_task_wdt_init(10, false);
  // esp_task_wdt_add(NULL);
}
void dummyBootSeq(float deg, float z_ht){
  coordinate start_pt,end_pt,p1_pt,p2_pt;
  // generate_trajec(deg, z_ht, &start_pt,&end_pt,&p1_pt,&p2_pt);
  gen_bz_pts(start_pt, p1_pt,p2_pt,end_pt,11, 1);
  Serial.println("Leg 1 done");
  rotate_points(120,&start_pt,&p1_pt,&p2_pt,&end_pt);
  Serial.println("Rotation for 2 done");
  gen_bz_pts(start_pt, p1_pt,p2_pt,end_pt,11, 2);
  Serial.println("Leg 2 done");
  rotate_points(180,&start_pt,&p1_pt,&p2_pt,&end_pt);
  Serial.println("Rotation for 0 done");
  gen_bz_pts(start_pt, p1_pt,p2_pt,end_pt,11, 0);
  Serial.println("Leg 0 done");
  Serial.println("Dummy Boot Seq :");
  boot_seq();
  int iteration = 0;
  while(iteration <10){
    iteration++;
    Serial.print("Iteration :");
    Serial.println(iteration);
    for(int i = 0; i<sizeof(bz_pts[0])/4; i++){
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print("Stack Space: ");
      Serial.println(uxHighWaterMark);
      Serial.println(i);
      set[0]->go_to(bz_pts[0][i][0], bz_pts[0][i][1], bz_pts[0][i][2]);
      // set[0]->go_to(0.0,180.0,-100.0);
      Serial.println(i);
      set[4]->go_to(bz_pts[1][i][0], bz_pts[1][i][1], bz_pts[1][i][2]);
      Serial.println(i);
      set[2]->go_to(bz_pts[2][i][0], bz_pts[2][i][1], bz_pts[2][i][2]);
      Serial.println(i+1);
      set[3]->go_to(lin_pts[0][10-i][0], lin_pts[0][10-i][1], lin_pts[0][10-i][2]);
      set[1]->go_to(lin_pts[1][10-i][0], lin_pts[1][10-i][1], lin_pts[1][10-i][2]);
      set[5]->go_to(lin_pts[2][10-i][0], lin_pts[2][10-i][1], lin_pts[2][10-i][2]);
      boot_seq();
      leg_state[0] = false;
      leg_state[0] = false;
      
    }
    for(int i = sizeof(bz_pts)/4-1; i>=0; i--){
      set[3]->go_to(bz_pts[0][i][0], bz_pts[0][i][1], bz_pts[0][i][2]);
      set[1]->go_to(bz_pts[1][i][0], bz_pts[1][i][1], bz_pts[1][i][2]);
      set[5]->go_to(bz_pts[2][i][0], bz_pts[2][i][1], bz_pts[2][i][2]);

      set[0]->go_to(lin_pts[0][10-i][0], lin_pts[0][10-i][1], lin_pts[0][10-i][2]);
      set[4]->go_to(lin_pts[1][10-i][0], lin_pts[1][10-i][1], lin_pts[1][10-i][2]);
      set[2]->go_to(lin_pts[2][10-i][0], lin_pts[2][10-i][1], lin_pts[2][10-i][2]);
    }
  }  
}

void loop() {
  int i = 0 ;
  if(Serial.available())
  {
    float deg = Serial.readStringUntil(' ').toFloat();
    float z_ht = Serial.readStringUntil('\r').toFloat();
    // dummyBootSeq(deg, z_ht);
    generate_trajec(deg, z_ht);
    // boot_seq();
  } 
  // for(int i=0;i<10;i++){
  //     float deg = i;
  //     float z_ht = 100.;
  //     generate_trajec(deg, z_ht);
  // }
}
