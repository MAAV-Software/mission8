
int main() {

	CameraInput cam(0);

	cv::Mat mat1;
	cv::Mat mat2;

	cam.getRGB(mat1);
	cam.getRGB(mat2);

	imshow(mat1);
	cv::waitKey(0);

	return 0;
}
