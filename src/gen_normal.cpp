///*************real-time reconstruction*********************/
#include <cmath>
#include "gen_normal.h"

void SamplePoints(const pcl::PointCloud<pcl::PointXYZ> &vCloud,
				  pcl::PointCloud<pcl::PointXYZ> &vNewCloud,
				  int iSampleNum,
				  bool bIntervalSamp = true)
{

	vNewCloud.clear();

	// sample by interval number
	if (bIntervalSamp)
	{

		for (int i = 0; i < vCloud.points.size(); i = i + iSampleNum)
			vNewCloud.push_back(vCloud.points[i]);

		// over the function and output
		return;

	} // end if

	// Sampling according to the given maximum total number
	// get the interval point number - how muny points should be skipped
	int iInterval = ceil(float(vCloud.points.size()) / float(iSampleNum));
	// sample
	for (int i = 0; i < vCloud.points.size(); i = i + iInterval)
		vNewCloud.push_back(vCloud.points[i]);

	// output
	return;
}

void translatePointRaw2PCL(float *pointX,
						   float *pointY,
						   float *pointZ,
						   pcl::PointCloud<pcl::PointXYZ> &vNewCloud,
						   int iCount)
{
	// printf("start translatePointRaw2PCL \n");

	for (int i = 0; i < iCount; i++)
	{
		pcl::PointXYZ newPoint;
		newPoint.x = pointX[i];
		newPoint.y = pointY[i];
		newPoint.z = pointZ[i];

		vNewCloud.push_back(newPoint);
	}
}

void translateNormalPCL2Raw(pcl::PointCloud<pcl::PointNormal> &vCloud,
							float *pRawCloud)
{
	// printf("start translateNormalPCL2Raw\n");
	// printf("vCloud.points.size() = %d\n", vCloud.points.size());

	for (int i = 0; i < vCloud.points.size(); i++)
	{
		// printf("%d-", i);
		pRawCloud[3 * i] = vCloud.points[i].normal_x;
		pRawCloud[3 * i + 1] = vCloud.points[i].normal_y;
		pRawCloud[3 * i + 2] = vCloud.points[i].normal_z;
	}
}

int gen_normal(float *pointX,
			   float *pointY,
			   float *pointZ,
			   float *pNormal,
			   const int iCount)
{
	// printf("start gen_normal\n");

	// std::vector<Point3D> vScenePoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pSceneCloud(new pcl::PointCloud<pcl::PointXYZ>);

	translatePointRaw2PCL(pointX, pointY, pointZ, *pSceneCloud, iCount);

	// printf("pSceneCloud->points[0].x = %f\n", pSceneCloud->points[0].x);
	// printf("pSceneCloud->points[0].y = %f\n", pSceneCloud->points[0].y);
	// printf("pSceneCloud->points[0].z = %f\n", pSceneCloud->points[0].z);

	// printf("pSceneCloud->points[100].x = %f\n", pSceneCloud->points[100].x);
	// printf("pSceneCloud->points[100].y = %f\n", pSceneCloud->points[100].y);
	// printf("pSceneCloud->points[100].z = %f\n", pSceneCloud->points[100].z);

	pcl::PointXYZ oViewPoint;
	// x 0.535947 y  0.62239 z 0.535947 bunny
	// x 0.457275 y  0.500000 z 1.814216 Cassette.las
	// x 0.0 -y 0.0 z 0.0 scene1oneframe.las
	oViewPoint.x = 0.0;
	oViewPoint.y = 0.0;
	oViewPoint.z = 0.0;

	// printf("start oExplicitBuilder\n");

	pcl::PointCloud<pcl::PointNormal>::Ptr pFramePNormal(new pcl::PointCloud<pcl::PointNormal>);

	ExplicitRec oExplicitBuilder;
	oExplicitBuilder.HorizontalSectorSize(12);
	oExplicitBuilder.SetViewPoint(oViewPoint);
	// printf("the size of pSceneCloud = %d", pSceneCloud->points.size());
	// printf("the size of pFramePNormal = %d", pFramePNormal->points.size());
	oExplicitBuilder.FrameReconstruction(*pSceneCloud, *pFramePNormal);

	translateNormalPCL2Raw(*pFramePNormal, pNormal);

	// printf("pNormal[0] = %f\n", pNormal[0]);
	// printf("pNormal[1] = %f\n", pNormal[1]);
	// printf("pNormal[2] = %f\n", pNormal[2]);

	return 0;
}