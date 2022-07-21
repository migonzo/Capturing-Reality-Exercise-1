#include "ply.h"

#include <fstream>

void write_ply_mesh(std::ostream &os, std::vector<Vertex> &vertices, std::vector<Face> &faces)
{
	os << "ply\nformat binary_little_endian 1.0\nelement vertex " << vertices.size() << "\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nelement face " << faces.size() << "\nproperty list uchar int vertex_indices\nend_header\n";
	for (int i = 0; i < vertices.size(); ++i) os.write((const char*)&vertices[i], 6 * 4 + 3);
	char three = 3;
	for (int i = 0; i < faces.size(); ++i) {
		os.write((const char*)&three, 1);
		os.write((const char*)&faces[i], 3 * 4);
	}
}

void write_ply_point_cloud(std::ostream& os, std::vector<Vertex_pc>& vertices)
{
	os << "ply\nformat binary_little_endian 1.0\nelement vertex " << vertices.size() << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
	for (int i = 0; i < vertices.size(); ++i) os.write((const char*)&vertices[i], 3 * 4);
}
