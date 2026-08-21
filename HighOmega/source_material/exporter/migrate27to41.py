outstr = """import os

def giveTexToObj(objName, texFileName):
	if texFileName == "placeholder.tga":
		return
	mat = bpy.data.materials.new(name=texFileName+"Mat")
	mat.use_nodes = True
	bsdf = mat.node_tree.nodes["Principled BSDF"]
	texImage = mat.node_tree.nodes.new('ShaderNodeTexImage')
	texImage.image = bpy.data.images.load(os.getcwd()+"\\\\"+texFileName)
	mat.node_tree.links.new(bsdf.inputs['Base Color'], texImage.outputs['Color'])
	# Assign it to object
	if bpy.data.objects[objName].data.materials:
		bpy.data.objects[objName].data.materials[0] = mat
	else:
		bpy.data.objects[objName].data.materials.append(mat)

"""

for i in bpy.data.objects:
	if hasattr(i, "game"):
		for j in i.game.properties:
			if type(j.value) is int:
				outstr += "bpy.data.objects[\""+i.name+"\"].__setitem__(\""+j.name+"\", int("+str(j.value)+"))\n"
			elif type(j.value) is float:
				outstr += "bpy.data.objects[\""+i.name+"\"].__setitem__(\""+j.name+"\", float("+str(j.value)+"))\n"
			else:
				outstr += "bpy.data.objects[\""+i.name+"\"].__setitem__(\""+j.name+"\", \""+str(j.value)+"\")\n"
	if i.data != None and i.data.name in bpy.data.meshes:
		try:
			textName = bpy.data.meshes[i.data.name].uv_textures[0].data[0].image.filepath.replace('\\', '/').split('/')[-1]
			outstr += "giveTexToObj(\""+i.name+"\",\""+textName+"\")\n"
		except:
			pass

import os
desktopPath = os.path.join(os.path.join(os.environ['USERPROFILE']), 'Desktop') 

f = open(desktopPath+"\\migrate_to_bl4.py", "w")
f.write(outstr)
f.close()