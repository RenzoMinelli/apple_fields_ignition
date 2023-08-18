#!/usr/bin/env python3
# Interfaz que permite generar, lanzar
# y grabar las instancias simuladas

from tkinter import *
from tkinter import ttk
import tkinter as tk
import os
from subprocess import Popen, run, PIPE
from tkinter.scrolledtext import ScrolledText
import base64
import json 

def launch_field(field):
    fields_ignition_path = run(['rospack', 'find', 'fields_ignition'], stdout=PIPE).stdout.decode("utf-8").replace('\n', '')
    if field in os.listdir(f'{fields_ignition_path}/generated'):
        try:
            p = Popen('bash -c "LC_ALL=C LANG=C roslaunch fields_ignition field.launch world_dir:=$(rospack find fields_ignition)/generated/{}/apple_field" &'.format(field),  shell=True) # something long running
            return "Done"
        except:
            return "Error" 

def record_bag(field, name, record_bag, record_video):
    global rosbag_proc

    fields_ignition_path = run(['rospack', 'find', 'fields_ignition'], stdout=PIPE)
    target = "{}/generated/{}/experiments/{}".format(fields_ignition_path.stdout.decode("utf-8").replace('\n', ''),field, name)

    print("Recording...")
    os.makedirs(target, exist_ok=True)
    if record_bag:
        rosbag_proc = Popen('nohup bash -c "rosbag record -O {}/record.bag -a -q __name:=record_bag"'.format(target), shell=True)
    
    if record_video:
        Popen('nohup bash -c "roslaunch fields_ignition record.launch field_name:={} experiment_name:={}"'.format(field, name), shell=True)
        
    return "Done"

def stop_record_bag():
    try:
        Popen('rosnode kill /record_bag',  shell=True)
        Popen('rosnode kill /record_images_node', shell=True)
        return "Done"
    except Exception as e:
        return "Error"
        
def generate(content, onGenerated):

    fields_ignition_path = run(['rospack', 'find', 'fields_ignition'], stdout=PIPE).stdout.decode("utf-8").replace('\n', '')
    params = ""
    if content.get("world_name"):
        params += " --world_name {}".format(content.get("world_name"))
    
    if content.get("field_id"):
        params += " --out_path {}/generated/{}".format(fields_ignition_path, content.get("field_id"))

    if content.get("row_count"):
        params += " --row_count {}".format(content.get("row_count"))
    
    if content.get("row_length"):
        params += " --row_length {}".format(content.get("row_length"))
    
    if content.get("row_dist"):
        params += " --row_dist {}".format(content.get("row_dist"))
    
    if content.get("crop_dist"):
        params += " --crop_dist {}".format(content.get("crop_dist"))
    
    if content.get("origin_coordinates"):
        params += " --origin_coordinates \"{}\"".format(content.get("origin_coordinates"))

    if content.get("tree_properties"):
        params += " --tree_properties {}".format(base64.urlsafe_b64encode(json.dumps(content.get("tree_properties")).encode()).decode())

    if content.get("camera").get("position"):
        params += " --camera_position \"{}\"".format(content.get("camera").get("position"))
    
    if content.get("husky").get("initial_position"):
        params += " --husky_initial_position \"{}\"".format(content.get("husky").get("initial_position"))
    
    # app.logger.info('python3 {}}/scripts/field_generator.py {}'.format(params))
    p = Popen('python3 {}/scripts/field_generator.py {}'.format(fields_ignition_path, params),  shell=True) # something long running
    # ... do other stuff while subprocess is running
    p.wait()

    onGenerated()
    return {}

def updateMenu(dropdown, dropdownVar, values):
    menu = dropdown['menu']

    # Clear the menu.
    menu.delete(0, 'end')
    for name in values:
        # Add menu items.
        menu.add_command(label=name, command=lambda name=name: dropdownVar.set(name))
        # OR menu.add_command(label=name, command=partial(var.set, name))

if __name__ == "__main__":

    fields_ignition_path = run(['rospack', 'find', 'fields_ignition'], stdout=PIPE).stdout.decode("utf-8").replace('\n', '')
    print(fields_ignition_path)
    root = Tk(className=' Gestor del Simulador  🍎')
    photo = tk.PhotoImage(file = f"{fields_ignition_path}/gui/apple_icon.png")
    root.wm_iconphoto(False, photo)
    # set window size
    root.geometry("800x800")
    frm = ttk.Frame(root)
    frm.grid()
    ttk.Label(frm, text="Ejecutar instancia del simulador", font=("Arial", 14), background="#000", foreground="#fff", padding="10").grid(column=1, row=0, columnspan = 2, sticky = tk.W+tk.E)
    sims_dropdownVariable = StringVar(root)
    sims_dropdownVariable.set(os.listdir(f'{fields_ignition_path}/generated')[0])
    sims_dropdown = OptionMenu(frm, sims_dropdownVariable, *os.listdir(f'{fields_ignition_path}/generated'))
    sims_dropdown.grid(column=1, row=1, sticky="ew")
    ttk.Button(frm, text="Launch", command=lambda: launch_field(sims_dropdownVariable.get())).grid(column=2, row=1, sticky="ew")

    ttk.Separator(frm, orient='horizontal').grid(row=2, columnspan=5, sticky="ew")
    
    ttk.Label(frm, text="Grabar ejecución", font=("Arial", 14), background="#000", foreground="#fff", padding="10").grid(column=1, row=3, columnspan = 2, sticky = tk.W+tk.E)
    
    name = StringVar(root, value='Nombre')
    nameTf = Entry(frm, textvariable=name).grid(column=1, row=4, columnspan = 2, sticky = tk.W+tk.E)
    record_bag_var = tk.BooleanVar()
    record_video_var = tk.BooleanVar()
    ttk.Checkbutton(frm, text="Video", variable=record_video_var).grid(row=5, column=1, sticky="ew")
    ttk.Checkbutton(frm, text="Bag", variable=record_bag_var).grid(row=6, column=1, sticky="ew")

    ttk.Button(frm, text="Grabar", command=lambda: record_bag(sims_dropdownVariable.get(), name.get(), record_bag_var.get(), record_video_var.get())).grid(column=2, row=5, sticky="ew")
    ttk.Button(frm, text="Parar", command=stop_record_bag).grid(column=2, row=6, sticky="ew")

    ttk.Separator(frm, orient='horizontal').grid(row=7, columnspan=5, sticky="ew")
    
    ttk.Label(frm, text="Generar campo", font=("Arial", 14), background="#000", foreground="#fff", padding="10").grid(column=1, row=8, columnspan = 2, sticky = tk.W+tk.E)
    
    field_props = StringVar(root)

    text_area = ScrolledText(frm, wrap=tk.WORD, 
                                      width=40, height=8,
                                      font=("Arial", 12))

    text_area.insert(tk.INSERT, """\
    {
        "world_name": "apple_field",
        "field_id": "sim_instance_1",
        "row_count": 1,
        "row_length": 5,
        "row_dist": 4,
        "crop_dist": 2,
        "origin_coordinates": "-2 0",
        "camera": {
            "position": "0.0012 0 1 0 -0.2618 1.57079632679"
        },
        "husky": {
            "initial_position": "-0.4 0 0 0 0 1.57"
        },
        "tree_properties": {
            "main": {
                "elements_offset_range": [0.5,0.6],
                "height_range": [2.2,2.3] 
            },
            "branches": {
                "count": 1,
                "radius_range": [0.05,0.07],
                "radius_decrease": 0.2,
                "brindillas": {
                    "offset_range": [0.5,0.6],
                    "length_range": [0.20, 1.0],
                    "distance_between": [0.02,0.03],
                    "radius_range": [0.006,0.01],
                    "radius_decrease": 0.2,
                    "element_offset": 0.1
                }
            },
            "fruits": {
                "count_proportion": 0.2,
                "size_range": [0.065, 0.07]
            },
            "leaves":   {
                "count_proportion": 0.3,
                "size_range": [0.1, 0.15]
            }     
        }
    }
    """)
  
    text_area.grid(column=1,columnspan = 2, row=9, pady=10, padx=10)

    ttk.Button(frm, text="Generar", command=lambda: generate(
        json.loads(text_area.get("1.0", tk.END)), 
        onGenerated=lambda: updateMenu(sims_dropdown, sims_dropdownVariable,  os.listdir(f'{fields_ignition_path}/generated')),
        )
    ).grid(column=2, row=10, sticky="ew")
    root.mainloop()
