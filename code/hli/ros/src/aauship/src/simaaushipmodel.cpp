/*
 * This node subscribes to a geometry_msgs/Transform topic from a matlab node
 * such that it can send a transform to be interpreted by rviz, this just
 * because I have no idea of sending the transforms directly from matabl, with
 * the ROS I/O toolbox for matlab.
 */

/* This node is the simulation model for aauship, it is supposed to be used as 
 * the Evolution function f().
 */

/* Matlab output
>> aaushipsimmodel(zeros(17,1), [1 0 0 0 0.1]')

ans =

   0.000765730547929
   0.000010722658911
   0.000384918776214
   0.000003812828575
   0.000545988159913
   0.000165918510655
   0.000922271810814
   0.007657401114737
   0.000100164427522
   0.005459881599134
   0.001659185106555
   0.009222718108145
   0.007657401114737
   0.000100164427522
   0.005459881599134
   0.001659185106555
   0.009222718108145
*/

#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>

#include <cmath>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

void adisCallback(const geometry_msgs::Transform::ConstPtr& msg)
{
  ROS_INFO("callback function");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "matlab_ahrs_helper_node");

  double dt = 0.05;
  double psi = 0.0;

  gsl_vector * xi = gsl_vector_alloc(17);
  gsl_matrix * Fxk = gsl_matrix_alloc(17,17);

  gsl_vector * ui = gsl_vector_alloc(5);
  gsl_matrix * Gammaxk = gsl_matrix_alloc(5,10);

  // Derivatives for N(t+1)
  gsl_matrix_set(Fxk, 0, 0, 1.0);
  gsl_matrix_set(Fxk, 0, 1, 0.0);
  gsl_matrix_set(Fxk, 0, 2, 0.0);
  gsl_matrix_set(Fxk, 0, 3, 0.0);
  gsl_matrix_set(Fxk, 0, 4, 0.0);
  gsl_matrix_set(Fxk, 0, 5, 0.0);
  gsl_matrix_set(Fxk, 0, 6, 0.0);
  gsl_matrix_set(Fxk, 0, 7, 0.0);
  gsl_matrix_set(Fxk, 0, 8, 0.0);
  gsl_matrix_set(Fxk, 0, 9, 0.0);
  gsl_matrix_set(Fxk, 0, 10, 0.0);
  gsl_matrix_set(Fxk, 0, 11, 0.0);
  gsl_matrix_set(Fxk, 0, 12, 0.0);
  gsl_matrix_set(Fxk, 0, 13, 0.0);
  gsl_matrix_set(Fxk, 0, 14, 0.0);
  gsl_matrix_set(Fxk, 0, 15, 0.0);
  gsl_matrix_set(Fxk, 0, 16, 0.0);
  // Derivatives for E(t+1)
  gsl_matrix_set(Fxk, 1, 0, 0.0);
  gsl_matrix_set(Fxk, 1, 1, 1.0);
  gsl_matrix_set(Fxk, 1, 2, 0.0);
  gsl_matrix_set(Fxk, 1, 3, 0.0);
  gsl_matrix_set(Fxk, 1, 4, 0.0);
  gsl_matrix_set(Fxk, 1, 5, 0.0);
  gsl_matrix_set(Fxk, 1, 6, 0.0);
  gsl_matrix_set(Fxk, 1, 7, 0.0);
  gsl_matrix_set(Fxk, 1, 8, 0.0);
  gsl_matrix_set(Fxk, 1, 9, 0.0);
  gsl_matrix_set(Fxk, 1, 10, 0.0);
  gsl_matrix_set(Fxk, 1, 11, 0.0);
  gsl_matrix_set(Fxk, 1, 12, 0.0);
  gsl_matrix_set(Fxk, 1, 13, 0.0);
  gsl_matrix_set(Fxk, 1, 14, 0.0);
  gsl_matrix_set(Fxk, 1, 15, 0.0);
  gsl_matrix_set(Fxk, 1, 16, 0.0);
  // Derivatives for x(t+1)
  gsl_matrix_set(Fxk, 2, 0, 0.0);
  gsl_matrix_set(Fxk, 2, 1, 0.0);
  gsl_matrix_set(Fxk, 2, 2, 1.0);
  gsl_matrix_set(Fxk, 2, 3, 0.0);
  gsl_matrix_set(Fxk, 2, 4, 0.0);
  gsl_matrix_set(Fxk, 2, 5, 0.0);
  gsl_matrix_set(Fxk, 2, 6, 0.0);
  gsl_matrix_set(Fxk, 2, 7, 0.0);
  gsl_matrix_set(Fxk, 2, 8, 0.0);
  gsl_matrix_set(Fxk, 2, 9, 0.0);
  gsl_matrix_set(Fxk, 2, 10, 0.0);
  gsl_matrix_set(Fxk, 2, 11, 0.0);
  gsl_matrix_set(Fxk, 2, 12, 0.0);
  gsl_matrix_set(Fxk, 2, 13, 0.0);
  gsl_matrix_set(Fxk, 2, 14, 0.0);
  gsl_matrix_set(Fxk, 2, 15, 0.0);
  gsl_matrix_set(Fxk, 2, 16, 0.0);
  // Derivatives for y(t+1)
  gsl_matrix_set(Fxk, 3, 0, 0.0);
  gsl_matrix_set(Fxk, 3, 1, 0.0);
  gsl_matrix_set(Fxk, 3, 2, 0.0);
  gsl_matrix_set(Fxk, 3, 3, 1.0);
  gsl_matrix_set(Fxk, 3, 4, 0.0);
  gsl_matrix_set(Fxk, 3, 5, 0.0);
  gsl_matrix_set(Fxk, 3, 6, 0.0);
  gsl_matrix_set(Fxk, 3, 7, 0.0);
  gsl_matrix_set(Fxk, 3, 8, 0.0);
  gsl_matrix_set(Fxk, 3, 9, 0.0);
  gsl_matrix_set(Fxk, 3, 10, 0.0);
  gsl_matrix_set(Fxk, 3, 11, 0.0);
  gsl_matrix_set(Fxk, 3, 12, 0.0);
  gsl_matrix_set(Fxk, 3, 13, 0.0);
  gsl_matrix_set(Fxk, 3, 14, 0.0);
  gsl_matrix_set(Fxk, 3, 15, 0.0);
  gsl_matrix_set(Fxk, 3, 16, 0.0);
  // Derivatives for phi(t+1)
  gsl_matrix_set(Fxk, 4, 0, 0.0);
  gsl_matrix_set(Fxk, 4, 1, 0.0);
  gsl_matrix_set(Fxk, 4, 2, -0.000047368775740);
  gsl_matrix_set(Fxk, 4, 3, 0.004282051934167);
  gsl_matrix_set(Fxk, 4, 4, 0.845057894901144);
  gsl_matrix_set(Fxk, 4, 5, -0.001585132859674);
  gsl_matrix_set(Fxk, 4, 6, -0.006037628365521);
  gsl_matrix_set(Fxk, 4, 7, -0.001634208621250);
  gsl_matrix_set(Fxk, 4, 8, 0.158428534679433);
  gsl_matrix_set(Fxk, 4, 9, -5.869213055434965);
  gsl_matrix_set(Fxk, 4, 10, -0.054820991730417);
  gsl_matrix_set(Fxk, 4, 11, -0.230859896547226);
  gsl_matrix_set(Fxk, 4, 12, 0.0);
  gsl_matrix_set(Fxk, 4, 13, 0.0);
  gsl_matrix_set(Fxk, 4, 14, 0.0);
  gsl_matrix_set(Fxk, 4, 15, 0.0);
  gsl_matrix_set(Fxk, 4, 16, 0.0);
  // Derivatives for theta(t+1)
  gsl_matrix_set(Fxk, 5, 0, 0.0);
  gsl_matrix_set(Fxk, 5, 1, 0.0);
  gsl_matrix_set(Fxk, 5, 2, -0.004009144592212);
  gsl_matrix_set(Fxk, 5, 3, 0.000822953153928);
  gsl_matrix_set(Fxk, 5, 4, -0.029965504425292);
  gsl_matrix_set(Fxk, 5, 5, 0.865852803887239);
  gsl_matrix_set(Fxk, 5, 6, -0.001299812197086);
  gsl_matrix_set(Fxk, 5, 7, -0.147577962531828);
  gsl_matrix_set(Fxk, 5, 8, 0.027659274325152);
  gsl_matrix_set(Fxk, 5, 9, -1.036331359780794);
  gsl_matrix_set(Fxk, 5, 10, -4.948665811403822);
  gsl_matrix_set(Fxk, 5, 11, -0.045698363336979);
  gsl_matrix_set(Fxk, 5, 12, 0.0);
  gsl_matrix_set(Fxk, 5, 13, 0.0);
  gsl_matrix_set(Fxk, 5, 14, 0.0);
  gsl_matrix_set(Fxk, 5, 15, 0.0);
  gsl_matrix_set(Fxk, 5, 16, 0.0);
  // Derivatives for psi(t+1)
  gsl_matrix_set(Fxk, 6, 0, 0.0);
  gsl_matrix_set(Fxk, 6, 1, 0.0);
  gsl_matrix_set(Fxk, 6, 2, 0.0);
  gsl_matrix_set(Fxk, 6, 3, 0.0);
  gsl_matrix_set(Fxk, 6, 4, 0.0);
  gsl_matrix_set(Fxk, 6, 5, 0.0);
  gsl_matrix_set(Fxk, 6, 6, 1.0);
  gsl_matrix_set(Fxk, 6, 7, 0.0);
  gsl_matrix_set(Fxk, 6, 8, 0.0);
  gsl_matrix_set(Fxk, 6, 9, 0.0);
  gsl_matrix_set(Fxk, 6, 10, 0.0);
  gsl_matrix_set(Fxk, 6, 11, 0.0);
  gsl_matrix_set(Fxk, 6, 12, 0.0);
  gsl_matrix_set(Fxk, 6, 13, 0.0);
  gsl_matrix_set(Fxk, 6, 14, 0.0);
  gsl_matrix_set(Fxk, 6, 15, 0.0);
  gsl_matrix_set(Fxk, 6, 16, 0.0);
  // Derivatives for u(t+1)
  gsl_matrix_set(Fxk, 7, 0, dt*cos(psi));
  gsl_matrix_set(Fxk, 7, 1, dt*sin(psi));
  gsl_matrix_set(Fxk, 7, 2,  0.049723406204869);
  gsl_matrix_set(Fxk, 7, 3,  0.000000533491234);
  gsl_matrix_set(Fxk, 7, 4, -0.000019426501116);
  gsl_matrix_set(Fxk, 7, 5, -0.000086975759482);
  gsl_matrix_set(Fxk, 7, 6, -0.000000842685134);
  gsl_matrix_set(Fxk, 7, 7,  0.988964802425052);
  gsl_matrix_set(Fxk, 7, 8,  0.000017884125782);
  gsl_matrix_set(Fxk, 7, 9, -0.000670202021289);
  gsl_matrix_set(Fxk, 7, 10, -0.003201606995903);
  gsl_matrix_set(Fxk, 7, 11, -0.000029556488868);
  gsl_matrix_set(Fxk, 7, 12,  0.988964802425052);
  gsl_matrix_set(Fxk, 7, 13,  0.000017884125782);
  gsl_matrix_set(Fxk, 7, 14, -0.000670202021289);
  gsl_matrix_set(Fxk, 7, 15, -0.003201606995903);
  gsl_matrix_set(Fxk, 7, 16, -0.000029556488868);
  // Derivatives for v(t+1)
  gsl_matrix_set(Fxk, 8, 0, dt*(-sin(psi)));
  gsl_matrix_set(Fxk, 8, 1, dt*cos(psi));
  gsl_matrix_set(Fxk, 8, 2,  0.000012127867596);
  gsl_matrix_set(Fxk, 8, 3,  0.045837548347937);
  gsl_matrix_set(Fxk, 8, 4,  0.039922222094321);
  gsl_matrix_set(Fxk, 8, 5,  0.000405862255594);
  gsl_matrix_set(Fxk, 8, 6, -0.000569417909974);
  gsl_matrix_set(Fxk, 8, 7,  0.000406627565023);
  gsl_matrix_set(Fxk, 8, 8,  0.840360099012158);
  gsl_matrix_set(Fxk, 8, 9,  1.477258087981388);
  gsl_matrix_set(Fxk, 8, 10,  0.013643189863155);
  gsl_matrix_set(Fxk, 8, 11, -0.024926387639480);
  gsl_matrix_set(Fxk, 8, 12,  0.000406627565023);
  gsl_matrix_set(Fxk, 8, 13,  0.840360099012158);
  gsl_matrix_set(Fxk, 8, 14,  1.477258087981388);
  gsl_matrix_set(Fxk, 8, 15,  0.013643189863155);
  gsl_matrix_set(Fxk, 8, 16, -0.024926387639480);
  // Derivatives for p(t+1)
  gsl_matrix_set(Fxk, 9, 0,                0.0);
  gsl_matrix_set(Fxk, 9, 1,                0.0);
  gsl_matrix_set(Fxk, 9, 2, -0.000001585666563);
  gsl_matrix_set(Fxk, 9, 3,  0.000141647982862);
  gsl_matrix_set(Fxk, 9, 4,  0.044921818677197);
  gsl_matrix_set(Fxk, 9, 5, -0.000053032470022);
  gsl_matrix_set(Fxk, 9, 6, -0.000196473915310);
  gsl_matrix_set(Fxk, 9, 7, -0.000073027846587);
  gsl_matrix_set(Fxk, 9, 8,  0.006788922250085);
  gsl_matrix_set(Fxk, 9, 9,  0.752899321745734);
  gsl_matrix_set(Fxk, 9, 10, -0.002445889774367);
  gsl_matrix_set(Fxk, 9, 11, -0.009625312696845);
  gsl_matrix_set(Fxk, 9, 12, -0.000073027846587);
  gsl_matrix_set(Fxk, 9, 13,  0.006788922250085);
  gsl_matrix_set(Fxk, 9, 14,  0.752899321745734);
  gsl_matrix_set(Fxk, 9, 15, -0.002445889774367);
  gsl_matrix_set(Fxk, 9, 16, -0.009625312696845);
  // Derivatives for q(t+1)
  gsl_matrix_set(Fxk, 10, 0,                0.0);
  gsl_matrix_set(Fxk, 10, 1,                0.0);
  gsl_matrix_set(Fxk, 10, 2, -0.000288466147122);
  gsl_matrix_set(Fxk, 10, 3,  0.000059754445484);
  gsl_matrix_set(Fxk, 10, 4, -0.002169255036631);
  gsl_matrix_set(Fxk, 10, 5,  0.040350145693658);
  gsl_matrix_set(Fxk, 10, 6, -0.000093936003883);
  gsl_matrix_set(Fxk, 10, 7, -0.012072492561263);
  gsl_matrix_set(Fxk, 10, 8,  0.002334197445616);
  gsl_matrix_set(Fxk, 10, 9, -0.086588459653789);
  gsl_matrix_set(Fxk, 10, 10,  0.595468162879000);
  gsl_matrix_set(Fxk, 10, 11, -0.003796674187051);
  gsl_matrix_set(Fxk, 10, 12, -0.012072492561263);
  gsl_matrix_set(Fxk, 10, 13,  0.002334197445616);
  gsl_matrix_set(Fxk, 10, 14, -0.086588459653789);
  gsl_matrix_set(Fxk, 10, 15,  0.595468162879000);
  gsl_matrix_set(Fxk, 10, 16, -0.003796674187051);
  // Derivatives for r(t+1)
  gsl_matrix_set(Fxk, 11, 0,                0.0);
  gsl_matrix_set(Fxk, 11, 1,                0.0);
  gsl_matrix_set(Fxk, 11, 2, -0.000000147096000);
  gsl_matrix_set(Fxk, 11, 3, -0.000004613815804);
  gsl_matrix_set(Fxk, 11, 4, -0.000454980362467);
  gsl_matrix_set(Fxk, 11, 5, -0.000004922258035);
  gsl_matrix_set(Fxk, 11, 6,  0.049680806806460);
  gsl_matrix_set(Fxk, 11, 7, -0.000005145503670);
  gsl_matrix_set(Fxk, 11, 8, -0.000202061143866);
  gsl_matrix_set(Fxk, 11, 9, -0.017394200825761);
  gsl_matrix_set(Fxk, 11, 10, -0.000172595493010);
  gsl_matrix_set(Fxk, 11, 11,  0.987291639691711);
  gsl_matrix_set(Fxk, 11, 12, -0.000005145503670);
  gsl_matrix_set(Fxk, 11, 13, -0.000202061143866);
  gsl_matrix_set(Fxk, 11, 14, -0.017394200825761);
  gsl_matrix_set(Fxk, 11, 15, -0.000172595493010);
  gsl_matrix_set(Fxk, 11, 16,  0.987291639691711);
  // Derivatives for dotu(t+1)
  gsl_matrix_set(Fxk, 12, 0, 0.0);
  gsl_matrix_set(Fxk, 12, 1, 0.0);
  gsl_matrix_set(Fxk, 12, 2, 0.0);
  gsl_matrix_set(Fxk, 12, 3, 0.0);
  gsl_matrix_set(Fxk, 12, 4, 0.0);
  gsl_matrix_set(Fxk, 12, 5, 0.0);
  gsl_matrix_set(Fxk, 12, 6, 0.0);
  gsl_matrix_set(Fxk, 12, 7, 0.0);
  gsl_matrix_set(Fxk, 12, 8, 0.0);
  gsl_matrix_set(Fxk, 12, 9, 0.0);
  gsl_matrix_set(Fxk, 12, 10, 0.0);
  gsl_matrix_set(Fxk, 12, 11, 0.0);
  gsl_matrix_set(Fxk, 12, 12, 1.0);
  gsl_matrix_set(Fxk, 12, 13, 0.0);
  gsl_matrix_set(Fxk, 12, 14, 0.0);
  gsl_matrix_set(Fxk, 12, 15, 0.0);
  gsl_matrix_set(Fxk, 12, 16, 0.0);
  // Derivatives for dotv(t+1)
  gsl_matrix_set(Fxk, 13, 0, 0.0);
  gsl_matrix_set(Fxk, 13, 1, 0.0);
  gsl_matrix_set(Fxk, 13, 2, 0.0);
  gsl_matrix_set(Fxk, 13, 3, 0.0);
  gsl_matrix_set(Fxk, 13, 4, 0.0);
  gsl_matrix_set(Fxk, 13, 5, 0.0);
  gsl_matrix_set(Fxk, 13, 6, 0.0);
  gsl_matrix_set(Fxk, 13, 7, 0.0);
  gsl_matrix_set(Fxk, 13, 8, 0.0);
  gsl_matrix_set(Fxk, 13, 9, 0.0);
  gsl_matrix_set(Fxk, 13, 10, 0.0);
  gsl_matrix_set(Fxk, 13, 11, 0.0);
  gsl_matrix_set(Fxk, 13, 12, 0.0);
  gsl_matrix_set(Fxk, 13, 13, 1.0);
  gsl_matrix_set(Fxk, 13, 14, 0.0);
  gsl_matrix_set(Fxk, 13, 15, 0.0);
  gsl_matrix_set(Fxk, 13, 16, 0.0);
  // Derivatives for dotp(t+1)
  gsl_matrix_set(Fxk, 14, 0, 0.0);
  gsl_matrix_set(Fxk, 14, 1, 0.0);
  gsl_matrix_set(Fxk, 14, 2, 0.0);
  gsl_matrix_set(Fxk, 14, 3, 0.0);
  gsl_matrix_set(Fxk, 14, 4, 0.0);
  gsl_matrix_set(Fxk, 14, 5, 0.0);
  gsl_matrix_set(Fxk, 14, 6, 0.0);
  gsl_matrix_set(Fxk, 14, 7, 0.0);
  gsl_matrix_set(Fxk, 14, 8, 0.0);
  gsl_matrix_set(Fxk, 14, 9, 0.0);
  gsl_matrix_set(Fxk, 14, 10, 0.0);
  gsl_matrix_set(Fxk, 14, 11, 0.0);
  gsl_matrix_set(Fxk, 14, 12, 0.0);
  gsl_matrix_set(Fxk, 14, 13, 0.0);
  gsl_matrix_set(Fxk, 14, 14, 1.0);
  gsl_matrix_set(Fxk, 14, 15, 0.0);
  gsl_matrix_set(Fxk, 14, 16, 0.0);;
  // Derivatives for dotq(t+1)
  gsl_matrix_set(Fxk, 15, 0, 0.0);
  gsl_matrix_set(Fxk, 15, 1, 0.0);
  gsl_matrix_set(Fxk, 15, 2, 0.0);
  gsl_matrix_set(Fxk, 15, 3, 0.0);
  gsl_matrix_set(Fxk, 15, 4, 0.0);
  gsl_matrix_set(Fxk, 15, 5, 0.0);
  gsl_matrix_set(Fxk, 15, 6, 0.0);
  gsl_matrix_set(Fxk, 15, 7, 0.0);
  gsl_matrix_set(Fxk, 15, 8, 0.0);
  gsl_matrix_set(Fxk, 15, 9, 0.0);
  gsl_matrix_set(Fxk, 15, 10, 0.0);
  gsl_matrix_set(Fxk, 15, 11, 0.0);
  gsl_matrix_set(Fxk, 15, 12, 0.0);
  gsl_matrix_set(Fxk, 15, 13, 0.0);
  gsl_matrix_set(Fxk, 15, 14, 0.0);
  gsl_matrix_set(Fxk, 15, 15, 1.0);
  gsl_matrix_set(Fxk, 15, 16, 0.0);
  // Derivatives for dotr(t+1)
  gsl_matrix_set(Fxk, 16, 0, 0.0);
  gsl_matrix_set(Fxk, 16, 1, 0.0);
  gsl_matrix_set(Fxk, 16, 2, 0.0);
  gsl_matrix_set(Fxk, 16, 3, 0.0);
  gsl_matrix_set(Fxk, 16, 4, 0.0);
  gsl_matrix_set(Fxk, 16, 5, 0.0);
  gsl_matrix_set(Fxk, 16, 6, 0.0);
  gsl_matrix_set(Fxk, 16, 7, 0.0);
  gsl_matrix_set(Fxk, 16, 8, 0.0);
  gsl_matrix_set(Fxk, 16, 9, 0.0);
  gsl_matrix_set(Fxk, 16, 10, 0.0);
  gsl_matrix_set(Fxk, 16, 11, 0.0);
  gsl_matrix_set(Fxk, 16, 12, 0.0);
  gsl_matrix_set(Fxk, 16, 13, 0.0);
  gsl_matrix_set(Fxk, 16, 14, 0.0);
  gsl_matrix_set(Fxk, 16, 15, 0.0);
  gsl_matrix_set(Fxk, 16, 16, 1.0);

  // Input
  gsl_matrix_set(Gammaxk, 0, 0, 0.000096711117179);
  gsl_matrix_set(Gammaxk, 0, 1, -0.000000186535396);
  gsl_matrix_set(Gammaxk, 0, 2, 0.000006792482908);
  gsl_matrix_set(Gammaxk, 0, 3, 0.000030411104714);
  gsl_matrix_set(Gammaxk, 0, 4, 0.000000294645152);
  gsl_matrix_set(Gammaxk, 0, 5, 0.003858460690541);
  gsl_matrix_set(Gammaxk, 0, 6, -0.000006253190833);
  gsl_matrix_set(Gammaxk, 0, 7,  0.000234336371080);
  gsl_matrix_set(Gammaxk, 0, 8, 0.001119443005560);
  gsl_matrix_set(Gammaxk, 0, 9, 0.000010334436667);

  gsl_matrix_set(Gammaxk, 1, 0, -0.000000178190958);
  gsl_matrix_set(Gammaxk, 1, 1, 0.000109392085769);
  gsl_matrix_set(Gammaxk, 1, 2, -0.000587684722957);
  gsl_matrix_set(Gammaxk, 1, 3, -0.000005963511340);
  gsl_matrix_set(Gammaxk, 1, 4, 0.000008410239369);
  gsl_matrix_set(Gammaxk, 1, 5, -0.000005789352247);
  gsl_matrix_set(Gammaxk, 1, 6, 0.004218981105503);
  gsl_matrix_set(Gammaxk, 1, 7, -0.021191577426855);
  gsl_matrix_set(Gammaxk, 1, 8, -0.000194285141910);
  gsl_matrix_set(Gammaxk, 1, 9, 0.000361582197916);

  gsl_matrix_set(Gammaxk, 2, 0, 0.000006792585715);
  gsl_matrix_set(Gammaxk, 2, 1, -0.000614037503466);
  gsl_matrix_set(Gammaxk, 2, 2, 0.022218381481424);
  gsl_matrix_set(Gammaxk, 2, 3, 0.000227304815257);
  gsl_matrix_set(Gammaxk, 2, 4, 0.000865783578858);
  gsl_matrix_set(Gammaxk, 2, 5, 0.000234342179255);
  gsl_matrix_set(Gammaxk, 2, 6, -0.022718328364035);
  gsl_matrix_set(Gammaxk, 2, 7, 0.841633167293072);
  gsl_matrix_set(Gammaxk, 2, 8, 0.007861218270394);
  gsl_matrix_set(Gammaxk, 2, 9, 0.033104837751983);

  gsl_matrix_set(Gammaxk, 3, 0, 0.000030411104714);
  gsl_matrix_set(Gammaxk, 3, 1, -0.000006242457453);
  gsl_matrix_set(Gammaxk, 3, 2, 0.000227301378617);
  gsl_matrix_set(Gammaxk, 3, 3, 0.001017564803224);
  gsl_matrix_set(Gammaxk, 3, 4, 0.000009859640610);
  gsl_matrix_set(Gammaxk, 3, 5, 0.001119443005560);
  gsl_matrix_set(Gammaxk, 3, 6, -0.000209807620670);
  gsl_matrix_set(Gammaxk, 3, 7, 0.007861023910662);
  gsl_matrix_set(Gammaxk, 3, 8, 0.037537781619914);
  gsl_matrix_set(Gammaxk, 3, 9, 0.000346641953348);

  gsl_matrix_set(Gammaxk, 4, 0, 0.000000293445738);
  gsl_matrix_set(Gammaxk, 4, 1, 0.000008740819169);
  gsl_matrix_set(Gammaxk, 4, 2, 0.000862003431982);
  gsl_matrix_set(Gammaxk, 4, 3, 0.000009819546481);
  gsl_matrix_set(Gammaxk, 4, 4, 0.001169460635756);
  gsl_matrix_set(Gammaxk, 4, 5, 0.000010266674632);
  gsl_matrix_set(Gammaxk, 4, 6, 0.000382199901606);
  gsl_matrix_set(Gammaxk, 4, 7, 0.032882581027846);
  gsl_matrix_set(Gammaxk, 4, 8, 0.000344374423143);
  gsl_matrix_set(Gammaxk, 4, 9, 0.046617623605528);


  psi = xi->data[11]*dt;
  
  gsl_blas_dgemv( CblasNoTrans, 1.0, Fxk, xi, 1.0, xi );
  gsl_blas_dgemv( CblasNoTrans, 1.0, Gammaxk, ui, 1.0, ui );
//	gsl_vector_add(  );

  printf("%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n", 
      xi->data[0] ,
      xi->data[1] ,
      xi->data[2] ,
      xi->data[3] ,
      xi->data[4] ,
      xi->data[5] ,
      xi->data[6] ,
      xi->data[7] ,
      xi->data[8] ,
      xi->data[9] ,
      xi->data[10],
      xi->data[11],
      xi->data[12],
      xi->data[13],
      xi->data[14],
      xi->data[15],
      xi->data[16]);

  ros::NodeHandle adis;
  ros::Subscriber adissub = adis.subscribe("test", 2, adisCallback);
//  ros::spin();

  return 0;
}
