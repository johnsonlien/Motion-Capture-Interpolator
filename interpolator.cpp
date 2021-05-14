#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

//Helper function to Linear interpolate vectors
vector Lerp(double t, vector& vStart, vector& vEnd) {
	return vStart * (1.0 - t) + vEnd * t;
}

// Function to print results out into a CSV file
void print2CSV(char * filename, Motion * pOutputMotion, int inputLength, int bone, int N) {
	FILE * fp;
	char fn[40];
	strcpy(fn, filename);
	if (bone == 0) strcat(fn, " (root)");
	else strcat(fn, " (femer)");
	
	if (N == 0) strcat(fn, " INPUT");
	else strcat(fn, " INTERP");
	
	strcat(fn, ".csv");
	
	fp = fopen(fn, "w");

	fprintf(fp, "%s %d\n", filename, bone);
	fprintf(fp, "frame, X, Y, Z\n");
	for (int frame = 1; frame < inputLength; frame++) {
		fprintf(fp, "%d, %f, %f, %f\n", frame,
			pOutputMotion->GetPosture(frame)->bone_rotation[bone].x(),
			pOutputMotion->GetPosture(frame)->bone_rotation[bone].y(),
			pOutputMotion->GetPosture(frame)->bone_rotation[bone].z());
	}
	fclose(fp);
}

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
  //FILE * fp;
  //fp = fopen("LinearEuler (root).csv", "w");

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

	  /*
	  fprintf(fp, "%d, %f, %f, %f\n", startKeyframe + frame,
		  interpolatedPosture.bone_rotation[0].x(),
		  interpolatedPosture.bone_rotation[0].y(),
		  interpolatedPosture.bone_rotation[0].z());
		  */
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }
    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

  print2CSV("LinearEuler", pOutputMotion, inputLength, 0, N);
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
	//convert angles to radians
	double theta1 = angles[0] * M_PI / 180.0;
	double theta2 = angles[1] * M_PI / 180.0;
	double theta3 = angles[2] * M_PI / 180.0;
	double temp[9] = { 0.0 };		// Used for the matrix multiplication

	// Define the three rotation matrices we will be multiplying
	double Rz[9] = {
		cos(theta3), -sin(theta3), 0,
		sin(theta3), cos(theta3), 0,
		0, 0, 1
	};
	double Ry[9] = {
		cos(theta2), 0, sin(theta2),
		0, 1, 0,
		-sin(theta2), 0, cos(theta2)
	};
	double Rx[9] = {
		1, 0, 0,
		0, cos(theta1), -sin(theta1),
		0, sin(theta1), cos(theta1)
	};

	// Matrix multiplication R = Rz * Ry * Rx
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			double sum = 0.0;
			for (int k = 0; k < 3; k++) {
				// Rz * Ry
				sum += Rz[i * 3 + k] * Ry[k * 3 + j];
			}
			temp[i * 3 + j] = sum;
		}
	}
	// R * Rx
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			double sum = 0.0;
			for (int k = 0; k < 3; k++) {
				//     (Rz*Ry)         * Rx
				sum += temp[i * 3 + k] * Rx[k * 3 + j];
			}
			R[i * 3 + j] = sum;
		}
	}
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames();
	int startKeyframe = 0;
	int i = 0;
	while (startKeyframe + N + 1 < inputLength) {
		int endKeyframe = startKeyframe + N + 1;

		int nextKeyframe = endKeyframe + N + 1;
		int prevKeyframe = startKeyframe - N - 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++) {
			Posture interpolatedPosture;
			vector root1, root2, ra_n, ra_nbar, rb_n;	// Used to interpolate root position
			
			double t = 1.0 * frame / (N + 1);

			// interpolate root position (Use bezier)
			root1 = startPosture->root_pos;
			root2 = endPosture->root_pos;

			// calculate a_n
			if (startKeyframe == 0) {
				Posture * pNext = pInputMotion->GetPosture(nextKeyframe);
				vector vNext = pNext->root_pos.p;
				ra_n = Lerp(1.0 / 3.0, root1, Lerp(2.0, vNext, root2));
			}
			else {
				Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
				vector vPrev = pPrev->root_pos.p;
				ra_nbar = Lerp(0.5, Lerp(2.0, vPrev, root1), root2);
				ra_n = Lerp(1.0 / 3.0, root1, ra_nbar);
			}

			// calculate b_n
			if (nextKeyframe > inputLength) {
				Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
				vector vPrev = pPrev->root_pos.p;
				rb_n = Lerp(1.0 / 3.0, root2, Lerp(2.0, vPrev, root1));
			}
			else {
				Posture * pNext = pInputMotion->GetPosture(nextKeyframe);
				vector vNext = pNext->root_pos.p;
				ra_nbar = Lerp(0.5, Lerp(2.0, root1, root2), vNext);
				rb_n = Lerp(-1.0 / 3.0, root2, ra_nbar);
			}
			
			interpolatedPosture.root_pos = DeCasteljauEuler(t, root1, ra_n, rb_n, root2);

			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				vector v1, v2, a_n, a_nbar, b_n;
				
				v1 = startPosture->bone_rotation[bone].p;
				v2 = endPosture->bone_rotation[bone].p;

				// Bezier control point: a_n
				if (startKeyframe == 0) {	// n = 1
					Posture * pNext = pInputMotion->GetPosture(nextKeyframe);
					vector vNext = pNext->bone_rotation[bone].p;
					//Linear Interpolation(t, start, end): start * (1 - t) + end * t 
					//a_1 = Slerp(q1, Slerp(q3, q2, 2.0), 1/3)
					a_n = Lerp(1.0 / 3.0, v1, Lerp(2.0, vNext, v2));
				}
				else {
					Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
					vector vPrev = pPrev->bone_rotation[bone].p;
					//Calculate a_nbar
					// a_nbar = Slerp(Slerp(qprev, q1, 2.0), q2, 0.5);
					a_nbar = Lerp(0.5, Lerp(2.0, vPrev, v1), v2);
					
					//a_n = Slerp(q1, a_nbar, 1.0/3)
					a_n = Lerp(1.0 / 3.0, v1, a_nbar);
				}
				
				// Bezier control point: b_n
				if (nextKeyframe > inputLength) { // n = N
					Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
					vector vPrev = pPrev->bone_rotation[bone].p;
					//b_N = Slerp(qN, Slerp(qprevprev, qprev, 2.0), 1.0/3)
					b_n = Lerp(1.0 / 3.0, v2, Lerp(2.0, vPrev, v1));
				}
				else {
					Posture * pNext = pInputMotion->GetPosture(nextKeyframe);
					vector vNext = pNext->bone_rotation[bone].p;
					//b_n = Slerp(qn, a_nbar, -1/3);
					a_nbar = Lerp(0.5, Lerp(2.0, v1, v2), vNext);
					//b_n = v2 * (4.0 / 3.0) + a_nbar * (-1.0 / 3.0);
					b_n = Lerp(-1.0 / 3.0, v2, a_nbar);
				}
				// Now we have our four points we can use decasteljau
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, v1, a_n, b_n, v2);
				
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
	}
	for (int frame = startKeyframe + 1; frame < inputLength; frame++) {
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
	print2CSV("BezierEuler", pOutputMotion, inputLength, 0, N);
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // Pretty much the same as other LinearInterpolationEuler function

	int inputLength = pInputMotion->GetNumFrames();
	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength) {
		int endKeyframe = startKeyframe + N + 1;
		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++) {
			Posture interpolatedPosture;
			
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			Quaternion<double> qStart, qEnd;
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				//interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;
				// We now convert the Euler angles to quaternion
				Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);

				// Perform Slerp and save those value and convert as Euler
				Quaternion2Euler(Slerp(t, qStart, qEnd), interpolatedPosture.bone_rotation[bone].p);
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++) {
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
	print2CSV("LinearQuaternion", pOutputMotion, inputLength, 0, N);
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames();
	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength) {
		int endKeyframe = startKeyframe + N + 1;	// End frame

		// Used to calculate previous and next keyframes
		int nextKeyframe = endKeyframe + N + 1;		// Next frame
		int prevKeyframe = startKeyframe - N - 1;	// Previous frame
		
		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			vector root1, root2, ra_n, ra_nbar, rb_n;
			root1 = startPosture->root_pos.p;
			root2 = endPosture->root_pos.p;
			
			// calculate a_n for the root
			if (startKeyframe == 0) {
				Posture * pNext = pInputMotion->GetPosture(nextKeyframe);
				vector vNext = pNext->root_pos.p;
				ra_n = Lerp(1.0 / 3.0, root1, Lerp(2.0, vNext, root2));
			}
			else {
				Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
				vector vPrev = pPrev->root_pos.p;
				ra_nbar = Lerp(0.5, Lerp(2.0, vPrev, root1), root2);
				ra_n = Lerp(1.0 / 3.0, root1, ra_nbar);
			}

			// calculate b_n for the root
			if (nextKeyframe > inputLength) {
				Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
				vector vPrev = pPrev->root_pos.p;
				rb_n = Lerp(1.0 / 3.0, root2, Lerp(2.0, vPrev, root1));
			}
			else {
				Posture * pNext = pInputMotion->GetPosture(nextKeyframe);
				vector vNext = pNext->root_pos.p;
				ra_nbar = Lerp(0.5, Lerp(2.0, root1, root2), vNext);
				rb_n = Lerp(-1.0 / 3.0, root2, ra_nbar);
			}

			interpolatedPosture.root_pos = DeCasteljauEuler(t, root1, ra_n, rb_n, root2);

			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				Quaternion<double> q1, q2, a_n, a_nbar, b_n;
				
				Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);
				
				// Bezier control point: a_n
				if (startKeyframe == 0) {	// calculating a_n will be different for n = 1
					Posture * pNext = pInputMotion->GetPosture(nextKeyframe);
					Quaternion<double> qNext;
					Euler2Quaternion(pNext->bone_rotation[bone].p, qNext);
					
					a_n = Slerp(1.0 / 3.0, q1, Double(qNext, q2));
				}
				else {
					// a_n, where 1 < n < N 
					Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
					Quaternion<double> qPrev;
					Euler2Quaternion(pPrev->bone_rotation[bone].p, qPrev);
					a_nbar = Slerp(0.5, Double(qPrev, q1), q2);
					a_n = Slerp(1.0 / 3.0, q1, a_nbar);
				}
				
				// Bezier control point: b_n
				if (nextKeyframe > inputLength) {	// b_n will be differnt for n = N
					Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
					Quaternion<double> qPrev;
					Euler2Quaternion(pPrev->bone_rotation[bone].p, qPrev);
					b_n = Slerp(1.0 / 3.0, q2, Double(qPrev, q1));
				}
				else {
					// b_n, where 1 < n < N
					Posture * pNext = pInputMotion->GetPosture(nextKeyframe);
					Quaternion<double> qNext; 
					Euler2Quaternion(pNext->bone_rotation[bone].p, qNext);
					a_nbar = Slerp(0.5, Double(q1, q2), qNext);
					b_n = Slerp(-1.0 / 3.0, q2, a_nbar);
				}

				Quaternion<double> interpolated = DeCasteljauQuaternion(t, q1, a_n, b_n, q2);
				double euler[3];
				Quaternion2Euler(interpolated, euler);
				interpolatedPosture.bone_rotation[bone] = euler;
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
		for (int frame = startKeyframe + 1; frame < inputLength; frame++) {
			pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
		}
	}
	print2CSV("BezierQuaternion", pOutputMotion, inputLength, 0, N);
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
	double R[9];
	Euler2Rotation(angles, R);
	q = Quaternion<double>::Matrix2Quaternion(R);
	q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
	double R[9];
	q.Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd)
{
	Quaternion<double> result;
	
	// cos(theta) = p * q
	double costheta = qStart.Gets() * qEnd.Gets() + qStart.Getx() * qEnd.Getx() + qStart.Gety() * qEnd.Gety() + qStart.Getz() * qEnd.Getz();
	double theta;
	if (costheta > 0) {
		theta = acos(costheta);
		// Check for divide by 0
		if (sin(theta) == 0) return qStart;
		result = (sin((1 - t) * theta) * qStart + sin(t * theta) * qEnd) / sin(theta);
	}
	else {
		theta = acos(-costheta);
		// Check for divide by 0
		if (sin(theta) == 0) return qStart;
		result = (sin((1 - t) * theta) * qStart - sin(t * theta) * qEnd) / sin(theta);
	}
	
	//This formula returns a unit quaternion so no need to normalize
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  //double(p,q) = 2(p*q)p - q
  Quaternion<double> result;
  
  result = Slerp(2.0, p, q);
  
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  vector result;
#if 0		
  double u[4] = { t*t*t, t*t, t, 1 };
  double Bezierbasis[16] = {
	  -1, 3, -3, 1,
	  3, -6, 3, 0,
	  -3, 3, 0, 0,
	  1, 0, 0, 0
  };
  double Beziercontrol[12] = {
	  p0.x(), p0.y(), p0.z(),
	  p1.x(), p1.y(), p1.z(),
	  p2.x(), p2.y(), p2.z(),
	  p3.x(), p3.y(), p3.z()
  };

  double temp[4];			// results in a 1x4 matrix
  for (int i = 0; i < 4; i++) {
	  double sum = 0.0;
	  for (int j = 0; j < 4; j++) {
		  sum += u[j] * Bezierbasis[j * 4 + i];
	  }
	  temp[i] = sum;
  }
  
  // 1x4 matrix multiply with 4x3
  for (int i = 0; i < 3; i++) {
	  double sum = 0.0;
	  for (int j = 0; j < 4; j++) {
		  sum += temp[j] * Beziercontrol[j * 3 + i];
	  }
	  result[i] = sum;
  }
#else
  vector q0 = p0 * (1 - t) + p1 * t;
  vector q1 = p1 * (1 - t) + p2 * t;
  vector q2 = p2 * (1 - t) + p3 * t;

  vector r0 = q0 * (1 - t) + q1 * t;
  vector r1 = q1 * (1 - t) + q2 * t;

  result = r0 * (1 - t) + r1 * t;
#endif

  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;		// P(t) = Slerp(R0, R1, t)
  
  Quaternion<double> q0 = Slerp(t, p0, p1);
  Quaternion<double> q1 = Slerp(t, p1, p2);
  Quaternion<double> q2 = Slerp(t, p2, p3);

  Quaternion<double> r0 = Slerp(t, q0, q1);
  Quaternion<double> r1 = Slerp(t, q1, q2);

  result = Slerp(t, r0, r1);		// result = P(t)

  return result;
}

