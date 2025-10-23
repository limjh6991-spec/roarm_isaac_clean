#
# Export Root Bodies and Objects Inside Root Groups to STL
#
# This is a FreeCAD script to export all visible root bodies and objects inside root groups in STL mesh format.
# Files will be stored inside an "exported_YYYY-MM-DD_HH-mm-ss" subfolder and named as "documentname_groupname_bodylabel.stl" or "documentname_bodylabel.stl".
#
import FreeCAD
import os.path
import datetime
doc = FreeCAD.activeDocument()
# Check if the document is saved (i.e., has a file name)
if not doc.FileName:
  FreeCAD.Console.PrintError("Document has not been saved. Please save the document first.\n")
  exit()
base_path = os.path.dirname(doc.FileName)
base_filename = os.path.splitext(os.path.basename(doc.FileName))[0]
# Get current date and time in the desired format
current_datetime = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
# Ensure the "exported_YYYY-MM-DD_HH-mm-ss" subfolder exists
export_folder = os.path.join(base_path, f"exported_{current_datetime}")
if not os.path.exists(export_folder):
  os.makedirs(export_folder)
def export_object_to_stl(obj, prefix=""):
  if hasattr(obj, 'Shape') and obj.ViewObject.Visibility:
    # Remove any non-allowed characters from the label for file naming
    sanitized_label = ''.join(e for e in obj.Label if e.isalnum() or e in ['_', '-'])
    filename = os.path.join(export_folder, base_filename + "_" + prefix + sanitized_label + ".stl")
    try:
      obj.Shape.exportStl(filename)
      FreeCAD.Console.PrintMessage(f"Exported {filename}\n")
    except Exception as e:
      FreeCAD.Console.PrintError(f"Error exporting {filename}: {str(e)}\n")
for obj in doc.Objects:
  # Check if the object is a root object (not used by any other objects)
  if not obj.InList:
    if obj.TypeId == 'App::DocumentObjectGroup':
      # If it's a root group, export objects inside
      for sub_obj in obj.Group:
        export_object_to_stl(sub_obj, prefix=obj.Label + "_")
    else:
      # If it's a single root body
      export_object_to_stl(obj)
