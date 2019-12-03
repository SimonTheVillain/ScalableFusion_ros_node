//
// Created by simon on 02.12.19.
//

#ifndef CATKIN_WS_RETREIVE_MESH_H
#define CATKIN_WS_RETREIVE_MESH_H

#include <video_source/source.h>
#include <gfx/camera.h>
#include <mesh_reconstruction.h>
#include <gfx/gpu_tex.h>
#include <cuda/test.h>
#include <scheduler.h>
#include <scheduler_threaded.h>
#include <utils/arcball.h>
#include <rendering/renderable_model.h>
#include <debug_render.h>
#include <gfx/garbage_collector.h>
#include <export/map_exporter.h>
#include <segmentation/incremental_segmentation.h>

#include "colored_mesh_msgs/RetreiveReconstruction.h"

#include <assimp/Exporter.hpp>
#include <assimp/scene.h>           // Output data structure
#include <assimp/version.h>

#include "colored_mesh_msgs/SendReconstruction.h"

using namespace std;
using namespace Eigen;


colored_mesh_msgs::SendReconstruction retreive(MeshReconstruction *map) {

	//todo: iterate over all patches download their data from gpu to cpu

	set<shared_ptr<DoubleStitch>> double_stitches;
	set<shared_ptr<TripleStitch>> triple_stitches;

	set<Triangle*> triangles;//don't think it will be needed
	vector<Vertex> vertices;
	vector<unsigned int> indices;
	std::map<int, unsigned int> start_indices;

	size_t triangle_count = 0;
	size_t vertex_count = 0;
	map->patches_mutex_.lock();
	for(pair<int, shared_ptr<MeshPatch>> id_patch : map->patches_) {//of course auto would be valid here as well
		shared_ptr<MeshPatch> patch = id_patch.second;
		start_indices[id_patch.first] = vertex_count;
		triangle_count += patch->triangles.size();
		vertex_count += patch->vertices.size();
		for(size_t i = 0; i < patch->double_stitches.size(); i++) {
			double_stitches.insert(patch->double_stitches[i]);
		}
		for(size_t i = 0; i < patch->triple_stitches.size(); i++) {
			triple_stitches.insert(patch->triple_stitches[i]);
		}
		//download the most current vertices and so on
	}
	for(auto double_stitch : double_stitches) {
		triangle_count += double_stitch->triangles.size();
	}
	for(auto triple_stitch : triple_stitches) {
		triangle_count += triple_stitch->triangles.size();
	}
	//how to store

	map->patches_mutex_.unlock();

	aiScene scene;

	scene.mRootNode = new aiNode();

	scene.mMaterials = new aiMaterial * [1];
	scene.mMaterials[0] = nullptr;
	scene.mNumMaterials = 1;
	scene.mMaterials[0] = new aiMaterial();

	scene.mMeshes = new aiMesh * [1];
	scene.mMeshes[0] = nullptr;
	scene.mNumMeshes = 1;
	scene.mMeshes[0] = new aiMesh();
	scene.mMeshes[0]->mMaterialIndex = 0;

	scene.mRootNode->mMeshes = new unsigned int[1];
	scene.mRootNode->mMeshes[0] = 0;
	scene.mRootNode->mNumMeshes = 1;
	auto p_mesh = scene.mMeshes[0];
	//lets just try a quad
	p_mesh->mVertices = new aiVector3D[vertex_count];
	p_mesh->mNumVertices = vertex_count;
	p_mesh->mTextureCoords[0] = new aiVector3D[vertex_count];
	p_mesh->mColors[0] = new aiColor4D[vertex_count];
	p_mesh->mFaces = new aiFace[triangle_count];
	p_mesh->mNumFaces = triangle_count;

	size_t j = 0;
	size_t k = 0;
	auto appendTrianglesAsFaces = [&k, &start_indices, &p_mesh] (
			vector<Triangle> triangles) {
		for(Triangle &triangle : triangles) {
			aiFace face;
			face.mIndices = new unsigned int[3];
			face.mNumIndices = 3;
			for(size_t l = 0; l < 3; l++) {
				unsigned int offset = start_indices[triangle.points[l].getPatch()->id];
				unsigned int index = triangle.points[l].getIndex() + offset;
				face.mIndices[l] = index;
			}
			p_mesh->mFaces[k]=face;
			k++;
		}
	};

	for(pair<int, shared_ptr<MeshPatch>> id_patch : map->patches_) {
		shared_ptr<MeshPatch> patch = id_patch.second;
		for(int i = 0 ; i < patch->vertices.size(); i++) {
			aiVector3D vec;
			vec.x = patch->vertices[i].p[0];
			vec.y = patch->vertices[i].p[1];
			vec.z = patch->vertices[i].p[2];
			p_mesh->mVertices[j] = vec;
			p_mesh->mColors[0][j].r = patch->vertex_colors[i][0];
			p_mesh->mColors[0][j].g = patch->vertex_colors[i][1];
			p_mesh->mColors[0][j].b = patch->vertex_colors[i][2];
			p_mesh->mColors[0][j].a = 255;
			vec.x = vec.x * 0.5f + 0.5f;
			vec.y = vec.y * 0.5f + 0.5f;
			vec.z = vec.z * 0.5f + 0.5f;
			p_mesh->mTextureCoords[0][j] = vec;
			j++;
		}
		appendTrianglesAsFaces(patch->triangles);
	}

	auto appendFaces = [&k, &start_indices, &p_mesh, &appendTrianglesAsFaces] (
			set<shared_ptr<GeometryBase>> &geoms) {
		for(auto geom : geoms) {
			appendTrianglesAsFaces(geom->triangles);
		}
	};
	//shared_ptr<GeometryBase> base = *(double_stitches.begin());
	appendFaces(reinterpret_cast<set<shared_ptr<GeometryBase>>&>(double_stitches));
	appendFaces(reinterpret_cast<set<shared_ptr<GeometryBase>>&>(triple_stitches));

	colored_mesh_msgs::SendReconstruction rec;

	for(int i=0;i<p_mesh->mNumFaces;i++){
		rec.request.mesh.triangles.emplace_back();
		rec.request.mesh.triangles.back().vertex_indices[0] = p_mesh->mFaces[i].mIndices[0];
		rec.request.mesh.triangles.back().vertex_indices[1] = p_mesh->mFaces[i].mIndices[1];
		rec.request.mesh.triangles.back().vertex_indices[2] = p_mesh->mFaces[i].mIndices[2];
	}
	for(int i=0;i<p_mesh->mNumVertices;i++){
		rec.request.mesh.vertices.emplace_back();
		rec.request.mesh.vertices.back().x = p_mesh->mVertices[i].x;
		rec.request.mesh.vertices.back().y = p_mesh->mVertices[i].y;
		rec.request.mesh.vertices.back().z = p_mesh->mVertices[i].z;
		rec.request.mesh.vertices.back().r = p_mesh->mColors[0][i].r;
		rec.request.mesh.vertices.back().g = p_mesh->mColors[0][i].g;
		rec.request.mesh.vertices.back().b = p_mesh->mColors[0][i].b;
		rec.request.mesh.vertices.back().a = p_mesh->mColors[0][i].a;
	}

	return rec;

	/*
	Assimp::Exporter exporter;
	aiReturn result = exporter.Export(&scene, "ply", file_path.c_str());
	if(result == aiReturn_SUCCESS) {
		cout << "file stored in " << file_path << endl;
	}else if(result == aiReturn_OUTOFMEMORY) {
		cout << "storing file " << file_path <<
			 " failed due to running out of memory" << endl;
	}else if(result == aiReturn_FAILURE) {
		cout << "storing file " << file_path << " failed" <<
			 exporter.GetErrorString() << endl;
	}
	 */
}


#endif //CATKIN_WS_RETREIVE_MESH_H
