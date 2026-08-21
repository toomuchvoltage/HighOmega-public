#!BPY

#
# Copyright (c) 2026 TooMuchVoltage Software Inc.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 

bl_info = {
    "name": "3MD Converter",
    "author": "Baktash Abdollah-Shamshir-saz",
    "version": (3, 0),
    "blender": (4, 1, 0),
    "location": "File > Export",
    "description": "Export to 3MD",
    "warning": "",
    "category": "Import-Export"}

import os
from os import path
import bpy
import math
import struct
import numpy
from math import *
from mathutils import *
from bpy.props import *
from bpy_extras.io_utils import ExportHelper
from pathlib import Path

_3MD_VERSION = ("HIGHOMEGA_3MD","3.0")

def GetTangentBiTangent (v,tex):
	A,B = Vector ([0,0,0]), Vector ([0,0,0])

	A = v[0]-v[1]
	B = v[2]-v[1]

	v1 = Vector ([0,0])
	v1 = tex[0]-tex[1]
	v2,v3 = Vector ([0,0]),Vector ([0,0])
	v2 = tex[2]-tex[1]
	origin,x_1,y_1 = Vector ([0,0,0]),Vector ([0,0,0]),Vector ([0,0,0])
	a,b = 0.0,0.0

	v3 = -tex[1]
	
	v1xv2y_v1yv2x = v1.x*v2.y - v1.y*v2.x
	inv_v1xv2y_v1yv2x = 0.00001
	if v1xv2y_v1yv2x != 0.0:
		inv_v1xv2y_v1yv2x = 1.0 / v1xv2y_v1yv2x;

	if v1xv2y_v1yv2x == 0.0:
		a = 1.0
	else:
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x

	if v2.x != 0.0:
		b = (v3.x - a*v1.x)/v2.x
	elif v2.y != 0.0:
		b = (v3.y - a*v1.y)/v2.y
	else:
		b = 1.0

	origin.x = v[1].x + a*A.x + b*B.x
	origin.y = v[1].y + a*A.y + b*B.y
	origin.z = v[1].z + a*A.z + b*B.z

	v3 = Vector((1.0, 0.0))-tex[1]
	if v1xv2y_v1yv2x == 0.0:
		a = 1.0
	else:
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x

	if v2.x != 0.0:
		b = (v3.x - a*v1.x)/v2.x
	elif v2.y != 0.0:
		b = (v3.y - a*v1.y)/v2.y
	else:
		b = 1.0

	x_1 = v[1] + a*A + b*B

	x_1_out = x_1 - origin

# ------------------------------------------------

	v3 = Vector((0.0, 1.0))-tex[1]
	if v1xv2y_v1yv2x == 0.0:
		a = 1.0
	else:
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x

	if v2.x != 0.0:
		b = (v3.x - a*v1.x)/v2.x
	elif v2.y != 0.0:
		b = (v3.y - a*v1.y)/v2.y
	else:
		b = 1.0

	y_1 = v[1] + a*A + b*B

	y_1_out = y_1 - origin

	return [x_1_out,y_1_out]

terrainExport = False

class mesh_output_object:
	outData = bytearray(b'')

	def __init__(self):
		self.outData = bytearray(b'')

	def write_dataline(self, inpTuple):
		counter = 0
		lastIndex = len(inpTuple)-1
		for i in inpTuple:
			if counter == lastIndex:
				lastItem = True
			else:
				lastItem = False
			counter += 1
			curType = type(i)
			if curType is int:
				self.outData += bytearray('I' if lastItem else 'i', 'latin-1')
				self.outData += bytearray(struct.pack ('I', i))
			elif curType is float:
				self.outData += bytearray('F' if lastItem else 'f', 'latin-1')
				self.outData += bytearray(struct.pack ('f', i))
			else:
				self.outData += bytearray('S' if lastItem else 's', 'latin-1')
				self.outData += bytearray(struct.pack ('I', len(i)))
				self.outData += bytearray(i, 'latin-1')

	def write_props (self, handle):
		props = []

		if handle.data != None and handle.data.name in bpy.data.meshes:
			try:
				props.append (('texname',bpy.data.meshes[handle.data.name].materials[0].node_tree.nodes["Image Texture"].image.filepath.replace('\\', '/').split('/')[-1]))
			except:
				props.append (('texname','placeholder.tga'))
				print ("Object %s has no texture assigned!\n" % (handle.data.name))
		else:
			props.append (('texname','placeholder.tga'))
			print ("Object %s has no texture assigned!\n" % (handle.name))

		for i in handle.items():
			if type(i[1]) is int or type(i[1]) is float or type(i[1]) is str:
				props.append (i)
			else:
				print ("Property %s is not integer, float or string, ignoring!\n" % (i[0]))

		if type(handle.instance_collection) == bpy.types.Collection:
			blendFileLink = handle.instance_collection.library.filepath.replace('\\','/').replace('//','/')
			if blendFileLink[0] == '/':
				blendFileLink = blendFileLink.replace('/','',1)
			modelFileLink = ".3md".join (blendFileLink.rsplit (".blend", 1))
			if not os.path.isfile(modelFileLink):
				raise Exception("Trying to link an unfound .3MD file: "+modelFileLink)
			modelFileBelong = "".join (blendFileLink.rsplit ("/", 1)[:-1])+"/"
			if modelFileBelong == "/":
				modelFileBelong = "./"
			props.append (("placedRelativeMesh", 		 modelFileLink))
			props.append (("placedRelativeMeshAssetLoc", modelFileBelong))
			props.append (("placedRelativeMeshTransform",handle.matrix_world[0][0], handle.matrix_world[0][1], handle.matrix_world[0][2], handle.matrix_world[0][3],\
														 handle.matrix_world[1][0], handle.matrix_world[1][1], handle.matrix_world[1][2], handle.matrix_world[1][3],\
														 handle.matrix_world[2][0], handle.matrix_world[2][1], handle.matrix_world[2][2], handle.matrix_world[2][3],\
														 handle.matrix_world[3][0], handle.matrix_world[3][1], handle.matrix_world[3][2], handle.matrix_world[3][3]))

		if len(props) > 0:
			self.write_dataline (('PROPS', 'rows', len (props)))
			for i in props:
				self.write_dataline (i)

def has_prop (handle, propName):
	return propName in handle.keys()

def get_prop (handle, propName):
	if propName in handle.keys():
		return handle.get(propName)
	return 0.0

def uintPackVec3(inC):
	return (((math.floor (inC[0] * 255.0)) | (math.floor (inC[1] * 255.0) << 8)) | (math.floor (inC[2] * 255.0) << 16))

def toZSignXY(inpVec):
	inpVecNorm = inpVec.normalized()
	retVal = ((int ((inpVecNorm.x + 1.0) * 16383.0) << 16) | int ((inpVecNorm.y + 1.0) * 32767.0));
	if inpVecNorm.z < 0.0:
		retVal |= 0x80000000
	return retVal

class mesh_geom:
	verts = []
	weights = []
	triIndices = []

	def __init__(self):
		self.verts = []
		self.weights = []
		self.triIndices = []

	def reserve(self,vertCount,triCount):
		self.verts = [None] * vertCount
		self.weights = [None] * vertCount
		self.triIndices = [None] * triCount

	def set_vert(self,idx,inp):
		self.verts[idx] = inp

	def set_weights(self,idx,inp):
		self.weights[idx] = inp

	def set_tri(self,idx,inp):
		self.triIndices[idx] = inp

	def get_min_max(self):
		min_v = Vector((self.verts[0][0]))
		max_v = Vector((self.verts[0][0]))
		for i in self.verts:
			min_v.x = min(i[0].x, min_v.x)
			min_v.y = min(i[0].y, min_v.y)
			min_v.z = min(i[0].z, min_v.z)
			max_v.x = max(i[0].x, max_v.x)
			max_v.y = max(i[0].y, max_v.y)
			max_v.z = max(i[0].z, max_v.z)
		return min_v, max_v

	def build_mesh(self, inpMesh, matWorld, matWorldDT, animated):
		count_verts = 0
		count_tris = 0
		for cur_poly in inpMesh.polygons:
			count_tris += len (cur_poly.vertices) - 2
		self.reserve (len(inpMesh.loops), count_tris)

		for cur_poly in inpMesh.polygons:
			for i in range (0, len (cur_poly.vertices)):
				loopIdx = cur_poly.loop_indices[i]
				edgeIdx = cur_poly.vertices[i]
				edge = matWorld @ inpMesh.vertices[edgeIdx].co
				edgeNorm = matWorldDT @ inpMesh.loops[loopIdx].normal

				if animated == True:
					edgeWeight = inpMesh.vertices[edgeIdx].groups
				else:
					edgeWeight = []

				try:
					edgeUV = inpMesh.uv_layers[0].data[loopIdx].uv
				except:
					edgeUV = Vector((0.0, 0.0))

				edgeCol = (0,0,0)
				if len (inpMesh.vertex_colors) > 0:
					edgeCol = (inpMesh.vertex_colors[0].data[loopIdx].color[0], inpMesh.vertex_colors[0].data[loopIdx].color[1], inpMesh.vertex_colors[0].data[loopIdx].color[2])

				self.set_vert (loopIdx, (edge, edgeCol, edgeUV, edgeNorm))
				if animated == True:
					self.set_weights (loopIdx, edgeWeight)

				count_verts += 1
				if count_verts % 200 == 0:
					print ("%d... " % (count_verts))

		count_tris = 0
		for cur_poly in inpMesh.polygons:
			tc = len (cur_poly.vertices)
			tri_vert_index = 0
			for i in range (0, tc - 2):
				vert_index1 = (tri_vert_index) % tc
				vert_index2 = (tri_vert_index+1) % tc
				if tri_vert_index > 0:
					vert_index3 = 0
				else:
					vert_index3 = (tri_vert_index+2) % tc

				self.set_tri(count_tris, (cur_poly.loop_indices[vert_index1], cur_poly.loop_indices[vert_index2], cur_poly.loop_indices[vert_index3]))
				count_tris += 1

				if tri_vert_index == 0:
					tri_vert_index += 2
				else:
					tri_vert_index += 1

	def write_compressed_verts(self, outputObject, instanceTransforms = None):
		preVertDataSize = 8 + len(self.triIndices) * 12
		paddingSize = 24 - (preVertDataSize % 24) # sizeof(RasterVertex)
		preVertDataSize += paddingSize
		outputObject.write_dataline (('TRIS','blob', preVertDataSize + len(self.verts) * 24))
		outputObject.outData += bytearray(struct.pack ('I', len(self.triIndices) ))
		outputObject.outData += bytearray(struct.pack ('I', len(self.verts) ))
		for i in self.triIndices:
			outputObject.outData += bytearray(struct.pack ('I', i[0] ))
			outputObject.outData += bytearray(struct.pack ('I', i[1] ))
			outputObject.outData += bytearray(struct.pack ('I', i[2] ))
		for i in range(0, paddingSize):
			outputObject.outData += b' '
		for i in self.verts:
			outputObject.outData += bytearray(struct.pack ('f', i[0].x ))
			outputObject.outData += bytearray(struct.pack ('f', i[0].y ))
			outputObject.outData += bytearray(struct.pack ('f', i[0].z ))
			outputObject.outData += bytearray(struct.pack ('I', uintPackVec3 (i[1]) ))
			outputObject.outData += bytearray(numpy.float16(i[2].x).view('e'))
			outputObject.outData += bytearray(numpy.float16(i[2].y).view('e'))
			outputObject.outData += bytearray(struct.pack ('I', toZSignXY (i[3]) ))
		if instanceTransforms != None:
			outputObject.write_dataline (('INSTANCES','blob', len(instanceTransforms) * 64))
			for i in instanceTransforms:
				for j in range (0, 16):
					outputObject.outData += bytearray(struct.pack ('f', i.matrix_world[int(j%4)][int(j/4)] ))

	def write_compressed_weights(self, parentArmatureLabel, outputObject):
		if len(self.weights) == 0:
			return
		outputObject.write_dataline (('WEIGHTS','blob',len(self.weights) * 16))
		for cur_vertex_weight_list in self.weights:
			vert_weights = []
			for cur_vertex_weight in cur_vertex_weight_list:
				vert_weights.append ((cur_vertex_weight.weight, cur_vertex_weight.group))
			vert_weights.sort (key=lambda tup: tup[0], reverse=True)
			vert_weights = vert_weights[:4]
			vert_weight_sum = 0
			for i in vert_weights:
				vert_weight_sum += i[0]
			vert_weight_sum = max(vert_weight_sum, 0.00001)
			for i in vert_weights:
				outputObject.outData += bytearray(struct.pack ('I', (i[1] << 16) | math.floor ((i[0] / vert_weight_sum) * 65535.0) ))
			if len(vert_weights) < 4:
				weight_gap = range (0, 4 - len(vert_weights))
				for i in weight_gap:
					outputObject.outData += bytearray(struct.pack ('I', 0xFFFF0000))
		outputObject.write_dataline (('PARENT_ARMATURE','rows',1))
		outputObject.write_dataline (tuple([parentArmatureLabel]))

def addTileReferences(meshMin, meshMax, objName, refTiles, worldMinMax):
	if "min" not in worldMinMax:
		worldMinMax["min"] = Vector((meshMin.x, meshMin.y, meshMin.z))
		worldMinMax["max"] = Vector((meshMax.x, meshMax.y, meshMax.z))
	else:
		worldMinMax["min"].x = min(worldMinMax["min"].x, meshMin.x)
		worldMinMax["min"].y = min(worldMinMax["min"].y, meshMin.y)
		worldMinMax["min"].z = min(worldMinMax["min"].z, meshMin.z)
		worldMinMax["max"].x = max(worldMinMax["max"].x, meshMax.x)
		worldMinMax["max"].y = max(worldMinMax["max"].y, meshMax.y)
		worldMinMax["max"].z = max(worldMinMax["max"].z, meshMax.z)
	meshMin /= 100.0
	meshMax /= 100.0
	meshMin.x = floor (meshMin.x)
	meshMin.y = floor (meshMin.y)
	meshMin.z = floor (meshMin.z)
	meshMax.x = floor (meshMax.x)
	meshMax.y = floor (meshMax.y)
	meshMax.z = floor (meshMax.z)
	for i in range(int (meshMin.x), int (meshMax.x) + 1):
		for j in range(int (meshMin.y), int (meshMax.y) + 1):
			for k in range(int (meshMin.z), int (meshMax.z) + 1):
				tileName = str(i)+"."+str(j)+"."+str(k)
				if tileName not in refTiles:
					refTiles[tileName] = []
				refTiles[tileName].append (objName)

def get_dir_transform(inp_world_transform):
	DT_c1 = Vector.cross (Vector((inp_world_transform[0][1], inp_world_transform[1][1], inp_world_transform[2][1])), Vector((inp_world_transform[0][2], inp_world_transform[1][2], inp_world_transform[2][2]))).normalized()
	DT_c2 = Vector.cross (Vector((inp_world_transform[0][2], inp_world_transform[1][2], inp_world_transform[2][2])), Vector((inp_world_transform[0][0], inp_world_transform[1][0], inp_world_transform[2][0]))).normalized()
	DT_c3 = Vector.cross (Vector((inp_world_transform[0][0], inp_world_transform[1][0], inp_world_transform[2][0])), Vector((inp_world_transform[0][1], inp_world_transform[1][1], inp_world_transform[2][1]))).normalized()
	inp_world_transform[0] = Vector((DT_c1.x, DT_c2.x, DT_c3.x, 0.0))
	inp_world_transform[1] = Vector((DT_c1.y, DT_c2.y, DT_c3.y, 0.0))
	inp_world_transform[2] = Vector((DT_c1.z, DT_c2.z, DT_c3.z, 0.0))
	inp_world_transform[3] = Vector((0.0    , 0.0    , 0.0    , 1.0))
	return inp_world_transform

def recursive_subobject_write(targetOutputMesh, parentObject, namePrefix, accumulatedMatrix):
	for subobject in parentObject.instance_collection.objects:
		if type(subobject.instance_collection) == bpy.types.Collection:
			print ("Recursively looking at: "+namePrefix+":"+subobject.name)
			recursive_subobject_write (targetOutputMesh, subobject.copy(), namePrefix+":"+subobject.name, accumulatedMatrix.copy() @ subobject.matrix_world.copy())
		else:
			print ("Writing mesh: "+namePrefix+":"+subobject.name)
			mat_world = accumulatedMatrix.copy() @ subobject.matrix_world.copy()
			mat_world_DT = get_dir_transform (mat_world.copy())

			cur_mesh = mesh_geom()
			cur_mesh.build_mesh(subobject.data, mat_world, mat_world_DT, False)

			targetOutputMesh.write_dataline (('MESH',namePrefix+":"+subobject.name))
			cur_mesh.write_compressed_verts(targetOutputMesh)

			targetOutputMesh.write_props (subobject)
			targetOutputMesh.write_dataline (tuple(["-1"]))

# https://iquilezles.org/articles/diskbbox/
def CylinderAABB(pa, pb, r):
	a = pb - pa
	sqrt_vec = Vector((1.0, 1.0, 1.0)) - ((a*a)/a.dot(a))
	sqrt_vec.x = sqrt(sqrt_vec.x)
	sqrt_vec.y = sqrt(sqrt_vec.y)
	sqrt_vec.z = sqrt(sqrt_vec.z)
	e = r * sqrt_vec;
	min1vec = pa - e
	min2vec = pb - e
	max1vec = pa + e
	max2vec = pb + e
	minvec = Vector((min(min1vec.x, min2vec.x), min(min1vec.y, min2vec.y), min(min1vec.z, min2vec.z)))
	maxvec = Vector((max(max1vec.x, max2vec.x), max(max1vec.y, max2vec.y), max(max1vec.z, max2vec.z)))
	return minvec, maxvec

def write_obj (fh):
	global terrainExport
	mainOutputMesh = mesh_output_object()
	mainOutputMesh.write_dataline (_3MD_VERSION)
	referenceTiles = None
	worldMinMax = None
	if terrainExport == True:
		referenceTiles = {}
		worldMinMax = {}

	num_frames = bpy.context.scene.frame_end

	all_bobbies = []
	guided_models = {}
	all_cameras = []
	written_instances = []
	instanced_meshes = {}

	processCount = 0
	cameraRailPrefix = '[CAMERA_RAIL]'
	guidedModelPathNumberPrefix = '[GUIDED_MODEL_PATH_NUMBER]'

	objectsToProcessCount = len(bpy.data.objects)

	if terrainExport == False:
		# Populate instanced mesh data
		print ("Commencing instance data collection:\n")
		for i in bpy.data.objects:
			if i.data == None or type(i.data) != bpy.types.Mesh or i.data in instanced_meshes:
				continue
			print ("Figuring out if "+i.name+" is an instance of something...\n")
			for j in bpy.data.objects:
				if j.data == None or type(j.data) != bpy.types.Mesh or j.data in instanced_meshes:
					continue
				if i.data == j.data and i.name != j.name:
					instanced_meshes[i.data] = []
					break
		for i in bpy.data.objects:
			if i.data == None or type(i.data) != bpy.types.Mesh or i.data not in instanced_meshes:
				continue
			print ("Collecting instance of type "+i.data.name+": "+i.name+"...\n")
			instanced_meshes[i.data].append (i)

	# Recursive mesh writes will write their own properties
	skipPropWriteAndClosure = False

	for cur_obj in bpy.data.objects:
		if cameraRailPrefix in cur_obj.name or guidedModelPathNumberPrefix in cur_obj.name:
			raise Exception ('Do not use reserved names '+guidedModelPathNumberPrefix+' or '+cameraRailPrefix+' in object names')
		if len(cur_obj.users_scene) == 0:
			print ("Processed object "+cur_obj.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))
			processCount += 1
			continue
		targetOutputMesh = None
		meshMin = Vector((0.0, 0.0, 0.0))
		meshMax = Vector((0.0, 0.0, 0.0))
		if terrainExport == True:
			targetOutputMesh = mesh_output_object()
			targetOutputMesh.write_dataline (_3MD_VERSION)
		else:
			targetOutputMesh = mainOutputMesh

		if type(cur_obj.instance_collection) == bpy.types.Collection:
			if has_prop (cur_obj, "bobby") or (cur_obj.parent != None and cur_obj.parent.type == "ARMATURE"):
				raise Exception ('Linked bobby or armatures are not meaningful. Linked assets bring in large structures like buildings.')
			# potentially nested instance_collections
			if terrainExport == False and not has_prop (cur_obj, "rigidBody") and not has_prop (cur_obj, "animatedModel") and not has_prop (cur_obj, "ragdoll"):
				recursive_subobject_write (targetOutputMesh, cur_obj, cur_obj.name, cur_obj.matrix_world.copy())
				skipPropWriteAndClosure = True
			else:
				curGroupMin = [0,0,0]
				curGroupMax = [0,0,0]
				curGroupSetOneVert = False
				for i in cur_obj.instance_collection.objects:
					for j in i.bound_box:
						Corner = i.matrix_world @ Vector((j[0], j[1], j[2]))
						if curGroupSetOneVert == False:
							curGroupMax[0] = curGroupMin[0] = Corner.x
							curGroupMax[1] = curGroupMin[1] = Corner.y
							curGroupMax[2] = curGroupMin[2] = Corner.z
							curGroupSetOneVert = True
						else:
							curGroupMin[0] = min (curGroupMin[0], Corner.x)
							curGroupMin[1] = min (curGroupMin[1], Corner.y)
							curGroupMin[2] = min (curGroupMin[2], Corner.z)
							curGroupMax[0] = max (curGroupMax[0], Corner.x)
							curGroupMax[1] = max (curGroupMax[1], Corner.y)
							curGroupMax[2] = max (curGroupMax[2], Corner.z)
				bpy.ops.mesh.primitive_cube_add(size=1, enter_editmode=False)
				addAABBCube = bpy.context.selected_objects[0]
				addAABBCube.scale = Vector(((curGroupMax[0] - curGroupMin[0]) * 0.5, (curGroupMax[1] - curGroupMin[1]) * 0.5, (curGroupMax[2] - curGroupMin[2]) * 0.5))
				addAABBCube.location = Vector(((curGroupMax[0] + curGroupMin[0]) * 0.5, (curGroupMax[1] + curGroupMin[1]) * 0.5, (curGroupMax[2] + curGroupMin[2]) * 0.5))
				bpy.ops.object.transform_apply(location=True, rotation=False, scale=True)

				mat_world = cur_obj.matrix_world.copy()
				mat_world_DT = get_dir_transform (cur_obj.matrix_world.copy())

				cur_mesh = mesh_geom()
				cur_mesh.build_mesh(addAABBCube.data, mat_world, mat_world_DT, False)

				targetOutputMesh.write_dataline (('MESH',cur_obj.name))
				cur_mesh.write_compressed_verts(targetOutputMesh)
				bpy.ops.object.delete ()
				meshMin, meshMax = cur_mesh.get_min_max ()

			print ("Processed object "+cur_obj.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))
		elif cur_obj.data.name in bpy.data.meshes:
			if has_prop (cur_obj, "bobby"):
				all_bobbies.append (cur_obj)
				continue
			if cur_obj.parent == None or cur_obj.parent.type != "ARMATURE":
				app_mesh = bpy.data.meshes[cur_obj.data.name]
				animated = False
				instanceReferenceMesh = False
				if cur_obj.data in instanced_meshes:
					if cur_obj != instanced_meshes[cur_obj.data][0]:
						processCount += 1
						continue
					else:
						instanceReferenceMesh = True
			else:
				instanceReferenceMesh = False
				bpy.context.scene.frame_set (0)
				dg = bpy.context.evaluated_depsgraph_get()
				object_eval = cur_obj.evaluated_get(dg)
				app_mesh = object_eval.to_mesh(preserve_all_data_layers=True, depsgraph=dg)
				animated = True

			cur_mesh = mesh_geom()
			if not instanceReferenceMesh:
				mat_world = cur_obj.matrix_world.copy()
				mat_world_DT = get_dir_transform (cur_obj.matrix_world.copy())
				cur_mesh.build_mesh(app_mesh, mat_world, mat_world_DT, animated)
			else:
				cur_mesh.build_mesh(app_mesh, Matrix(), Matrix(), animated)

			targetOutputMesh.write_dataline (('MESH',cur_obj.name))
			if not instanceReferenceMesh:
				cur_mesh.write_compressed_verts(targetOutputMesh)
			else:
				cur_mesh.write_compressed_verts(targetOutputMesh, instanced_meshes[cur_obj.data])
			if animated == True:
				cur_mesh.write_compressed_weights(cur_obj.parent.name, targetOutputMesh)
				object_eval.to_mesh_clear()

			if terrainExport == True:
				meshMin, meshMax = cur_mesh.get_min_max ()

			print ("Processed object "+cur_obj.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))
		elif cur_obj.data.name in bpy.data.lights:
			lampDir = cur_obj.rotation_euler.to_matrix() @ Vector((0,0,-1))
			lightObject = bpy.data.lights[cur_obj.data.name]
			if terrainExport == True:
				if has_prop (cur_obj, "guidedModelPathNumber"):
					guidedModelPathNumber = get_prop (cur_obj, "guidedModelPathNumber")
					if guidedModelPathNumber not in guided_models:
						guided_models[guidedModelPathNumber] = []
					guided_models[guidedModelPathNumber].append(cur_obj)
					continue
				elif has_prop (cur_obj, "windSource"):
					meshMin, meshMax = CylinderAABB (cur_obj.location, cur_obj.location + lampDir * lightObject.cutoff_distance, lightObject.shadow_soft_size)
				else:
					meshMin = Vector (cur_obj.location)
					meshMax = Vector (cur_obj.location)
			targetOutputMesh.write_dataline ((cur_obj.type,cur_obj.name))
			targetOutputMesh.write_dataline (('DESCRIPTION','rows',6))
			targetOutputMesh.write_dataline (('pos',cur_obj.location.x,cur_obj.location.y,cur_obj.location.z))
			targetOutputMesh.write_dataline (('col',lightObject.color.r,lightObject.color.g,lightObject.color.b))
			targetOutputMesh.write_dataline (('dir',lampDir.x,lampDir.y,lampDir.z))
			targetOutputMesh.write_dataline (('dist',lightObject.cutoff_distance))
			targetOutputMesh.write_dataline (('enrg',lightObject.energy))
			targetOutputMesh.write_dataline (('rad',lightObject.shadow_soft_size))
			print ("Processed object "+cur_obj.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))
		elif cur_obj.data.name in bpy.data.cameras:
			if terrainExport == True:
				all_cameras.append (cur_obj)
				continue
			cameraObject = bpy.data.cameras[cur_obj.data.name]
			targetOutputMesh.write_dataline ((cur_obj.type,cur_obj.name))
			targetOutputMesh.write_dataline (('DESCRIPTION','rows',2))
			targetOutputMesh.write_dataline (('pos',cur_obj.location.x,cur_obj.location.y,cur_obj.location.z))
			cameraDir = cur_obj.rotation_euler.to_matrix() @ Vector((0,0,-1))
			targetOutputMesh.write_dataline (('dir',cameraDir.x,cameraDir.y,cameraDir.z))
			print ("Processed object "+cur_obj.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))
		elif cur_obj.data.name in bpy.data.armatures:
			if terrainExport == True:
				meshMin = Vector (cur_obj.location)
				meshMax = Vector (cur_obj.location)
			targetOutputMesh.write_dataline ((cur_obj.type,cur_obj.name))
			keyframes = []
			for action in bpy.data.actions:
				for fcu in action.fcurves:
					for keyframe in fcu.keyframe_points:
						if keyframe.co.x not in keyframes:
							keyframes.append (keyframe.co.x)
			list.sort (keyframes)
			if len (keyframes) < 2 or keyframes[0] != 0:
				raise Exception("You need at least 2 keyframes with the first one being zero") 

			keyFrameData = bytearray(struct.pack ('I', len(cur_obj.pose.bones)))
			for cur_bone in cur_obj.pose.bones:
				keyFrameData += bytearray(struct.pack ('I', len(cur_bone.name)))
				keyFrameData += str.encode(cur_bone.name)
			keyFrameData += bytearray(struct.pack ('I', len(keyframes)))
			for i in keyframes:
				bpy.context.scene.frame_set (int(i))
				keyFrameData += bytearray(struct.pack ('I', int(i)))
				for cur_bone in cur_obj.pose.bones:
					cur_bone_mat = cur_obj.matrix_world @ cur_bone.matrix;
					for j in range(0, 16):
						keyFrameData += bytearray(struct.pack ('f', cur_bone_mat[int(j%4)][int(j/4)]))
			targetOutputMesh.write_dataline (('KEYFRAME_DATA','blob',len(keyFrameData)))
			targetOutputMesh.outData += keyFrameData
		else:
			if terrainExport == True:
				meshMin = Vector (cur_obj.location)
				meshMax = Vector (cur_obj.location)
			targetOutputMesh.write_dataline ((cur_obj.type,cur_obj.name))
			targetOutputMesh.write_dataline (('DESCRIPTION','rows',1))
			targetOutputMesh.write_dataline (('pos',cur_obj.location.x,cur_obj.location.y,cur_obj.location.z))
			print ("Processed object "+cur_obj.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))

		processCount += 1

		if skipPropWriteAndClosure == False:
			targetOutputMesh.write_props (cur_obj)
			targetOutputMesh.write_dataline (tuple(["-1"]))
		else:
			skipPropWriteAndClosure = False

		if terrainExport == True:
			if path.isfile(cur_obj.name+".blend"):
				raise Exception("Blend file already exists for this object name. Please rename this object: "+cur_obj.name)
			targetMeshFile = open(cur_obj.name+".3md", "wb")
			targetMeshFile.write (targetOutputMesh.outData)
			targetMeshFile.close ()

			addTileReferences (meshMin, meshMax, cur_obj.name, referenceTiles, worldMinMax)

	if terrainExport == True:
		for guidedModelPathNumber in guided_models:
			targetOutputMesh = mesh_output_object()
			targetOutputMesh.write_dataline (_3MD_VERSION)
			looked_at_one_point = False
			for cur_obj in guided_models[guidedModelPathNumber]:
				lightObject = bpy.data.lights[cur_obj.data.name]
				targetOutputMesh.write_dataline ((cur_obj.type,cur_obj.name))
				targetOutputMesh.write_dataline (('DESCRIPTION','rows',6))
				targetOutputMesh.write_dataline (('pos',cur_obj.location.x,cur_obj.location.y,cur_obj.location.z))
				targetOutputMesh.write_dataline (('col',lightObject.color.r,lightObject.color.g,lightObject.color.b))
				lampDir = cur_obj.rotation_euler.to_matrix() @ Vector((0,0,-1))
				targetOutputMesh.write_dataline (('dir',lampDir.x,lampDir.y,lampDir.z))
				targetOutputMesh.write_dataline (('dist',lightObject.cutoff_distance))
				targetOutputMesh.write_dataline (('enrg',lightObject.energy))
				targetOutputMesh.write_dataline (('rad',lightObject.shadow_soft_size))
				targetOutputMesh.write_props (cur_obj)
				targetOutputMesh.write_dataline (tuple(["-1"]))
				if not looked_at_one_point:
					meshMin = Vector(cur_obj.location)
					meshMax = Vector(cur_obj.location)
					looked_at_one_point = True
				else:
					meshMinX = min (cur_obj.location.x, meshMin.x)
					meshMinY = min (cur_obj.location.y, meshMin.y)
					meshMinZ = min (cur_obj.location.z, meshMin.z)
					meshMaxX = max (cur_obj.location.x, meshMax.x)
					meshMaxY = max (cur_obj.location.y, meshMax.y)
					meshMaxZ = max (cur_obj.location.z, meshMax.z)
					meshMin = Vector((meshMinX, meshMinY, meshMinZ))
					meshMax = Vector((meshMaxX, meshMaxY, meshMaxZ))
				print ("Processed object "+cur_obj.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))
				processCount += 1
			targetMeshFile = open(guidedModelPathNumberPrefix+str(guidedModelPathNumber)+".3md", "wb")
			targetMeshFile.write (targetOutputMesh.outData)
			targetMeshFile.close ()
			addTileReferences (meshMin, meshMax, guidedModelPathNumberPrefix+str(guidedModelPathNumber), referenceTiles, worldMinMax)

		if len(all_cameras) > 0:
			targetOutputMesh = mesh_output_object()
			targetOutputMesh.write_dataline (_3MD_VERSION)
			looked_at_one_point = False
			for cur_obj in all_cameras:
				cameraObject = bpy.data.cameras[cur_obj.data.name]
				targetOutputMesh.write_dataline ((cur_obj.type,cur_obj.name))
				targetOutputMesh.write_dataline (('DESCRIPTION','rows',2))
				targetOutputMesh.write_dataline (('pos',cur_obj.location.x,cur_obj.location.y,cur_obj.location.z))
				cameraDir = cur_obj.rotation_euler.to_matrix() @ Vector((0,0,-1))
				targetOutputMesh.write_dataline (('dir',cameraDir.x,cameraDir.y,cameraDir.z))
				targetOutputMesh.write_props (cur_obj)
				targetOutputMesh.write_dataline (tuple(["-1"]))
				if not looked_at_one_point:
					meshMin = Vector(cur_obj.location)
					meshMax = Vector(cur_obj.location)
					looked_at_one_point = True
				else:
					meshMinX = min (cur_obj.location.x, meshMin.x)
					meshMinY = min (cur_obj.location.y, meshMin.y)
					meshMinZ = min (cur_obj.location.z, meshMin.z)
					meshMaxX = max (cur_obj.location.x, meshMax.x)
					meshMaxY = max (cur_obj.location.y, meshMax.y)
					meshMaxZ = max (cur_obj.location.z, meshMax.z)
					meshMin = Vector((meshMinX, meshMinY, meshMinZ))
					meshMax = Vector((meshMaxX, meshMaxY, meshMaxZ))
				print ("Processed object "+cur_obj.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))
				processCount += 1
			targetMeshFile = open(cameraRailPrefix+".3md", "wb")
			targetMeshFile.write (targetOutputMesh.outData)
			targetMeshFile.close ()
			addTileReferences (meshMin, meshMax, cameraRailPrefix, referenceTiles, worldMinMax)

		mainOutputMesh.write_dataline (('ZONES', 'allZones'))
		mainOutputMesh.write_dataline (('WORLDBOUNDS', 'rows', 1))
		mainOutputMesh.write_dataline ((worldMinMax["min"].x, worldMinMax["min"].y, worldMinMax["min"].z, worldMinMax["max"].x, worldMinMax["max"].y, worldMinMax["max"].z))
		mainOutputMesh.write_dataline (('DESCRIPTION', 'rows', len(referenceTiles)))
		for i in referenceTiles:
			curZoneRefs = (i, len(referenceTiles[i]))
			for j in referenceTiles[i]:
				curZoneRefs += (j,)
			mainOutputMesh.write_dataline (curZoneRefs)
		mainOutputMesh.write_dataline (tuple(["-1"]))
	else:
		if len (all_bobbies) > 0:
			mainOutputMesh.write_dataline (('BOBBIES', 'allBobbies'))
			mainOutputMesh.write_dataline (('DESCRIPTION', 'rows', len (all_bobbies) + 1))
			mainOutputMesh.write_dataline (('rad',all_bobbies[0].matrix_world[0][0] * 2.0))
			for i in all_bobbies:
				mainOutputMesh.write_dataline ((i.matrix_world[0][3], i.matrix_world[1][3], i.matrix_world[2][3]))
				print ("Processed object "+i.name+" ... %d of %d\n" % ((processCount+1), objectsToProcessCount))
				processCount += 1
			mainOutputMesh.write_dataline (tuple(["-1"]))
	fh.write(mainOutputMesh.outData)

	print ("Done with export!")

class _3MDConverter(bpy.types.Operator, ExportHelper):
	bl_idname = "export.3md"
	bl_label = "Export 3MD"
	bl_options = {'PRESET', 'UNDO'}
	filename_ext = ".3md"
	
	filepath: StringProperty(subtype='FILE_PATH')
	terrainExportOption: BoolProperty(name="Terrain export", description="This will break down and export world chunks into separate files and create a zone directory", default=False)

	def execute(self, context):
		global terrainExport
		FilePath = bpy.path.ensure_ext(self.filepath, ".3md")

		terrainExport = self.terrainExportOption

		fh = open(FilePath if not terrainExport else "zones.3md", "wb")
		write_obj (fh)
		fh.close ()

		for listNames in Path(".").glob("*.matdf"):
			listNames.unlink()

		return {"FINISHED"}

	def invoke(self, context, event):
		return ExportHelper.invoke(self, context, event)

def menu_func(self, context):
	self.layout.operator(_3MDConverter.bl_idname, text="3MD (.3MD)")

def register():
	bpy.utils.register_class(_3MDConverter)
	bpy.types.TOPBAR_MT_file_export.append(menu_func)

def unregister():
	bpy.utils.unregister_class(_3MDConverter)
	bpy.types.TOPBAR_MT_file_export.remove(menu_func)

if __name__ == "__main__":
	register()