#include "gcs/GCSConsts.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;
namespace maav
{
namespace gcs
{
void GCSConsts::writeGains(array<double, 21> gains) const
{
	ofstream fout(FILENAME);
	fout << fixed << setprecision(2) << "---" << endl;
	fout << "# Config file for GCS" << endl;
	fout << "# Tuning parameters to be loaded on launch" << endl;
	fout << "Tuning:\n  pos:" << endl;
	fout << "    x: [" << gains[0] << ", " << gains[1] << ", " << gains[2] << "]" << endl;
	fout << "    y: [" << gains[3] << ", " << gains[4] << ", " << gains[5] << "]" << endl;
	fout << "    z: [" << gains[6] << ", " << gains[7] << ", " << gains[8] << "]" << endl;
	fout << "    yaw: [" << gains[9] << ", " << gains[10] << ", " << gains[11] << "]" << endl;
	fout << "  rate:" << endl;
	fout << "    x: [" << gains[12] << ", " << gains[13] << ", " << gains[14] << "]" << endl;
	fout << "    y: [" << gains[15] << ", " << gains[16] << ", " << gains[17] << "]" << endl;
	fout << "    z: [" << gains[18] << ", " << gains[19] << ", " << gains[20] << "]" << endl;
	fout << "# GUI Spacing\nSpacing:" << endl;
	fout << "  small: " << SMALL_SPACE << endl;
	fout << "  med: " << MED_SPACE << endl;
	fout << "  large: " << LARGE_SPACE << endl;
	fout << "# Status timeouts\nTimeout:" << endl;
	fout << "  quiet: " << QUIET_TIMEOUT << endl;
	fout << "  down: " << DOWN_TIMEOUT << endl;
	fout << "# ZCM url" << endl;
	fout << "url: \"" << URL << "\"" << endl;
	fout.close();
}

}  // namespace gcs
}  // namespace maav
