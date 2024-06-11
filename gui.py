import time
import tkinter
from tkinter import ttk
import tkinter.messagebox
import argparse
from tkinter.messagebox import showinfo

from gantry_ctrl import OpenBuildsGantryController


class LabelEntryFrame(ttk.Frame):
    """
    a custom ttk.Frame module consists of a ttk.Label and a ttk.Entry.
    """
    def __init__(self, frame_params, label_params={}, entry_params={}, entry_default=''):
        super().__init__(**frame_params)

        self.label = ttk.Label(master=self, **label_params)
        self.entry = ttk.Entry(master=self, **entry_params)
        self.entry.insert(0, entry_default)
        
        self.label.grid(row=0, column=0)
        self.entry.grid(row=0, column=1)

        self.pack()
        
class ParaFrame(ttk.LabelFrame):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # preamble and postamble
        # self.unit = LabelEntryFrame(frame_params={'master': self},
        #                              label_params={'text': 'Unit'},
        #                              entry_params={'width': 10},
        #                              entry_default='Unit')

        self.unit = ttk.Frame(master=self)
        self.unit_label = ttk.Label(master=self.unit, text='Unit')
        self.unit_label.grid(row=0, column=0)
        self.unit_var = tkinter.StringVar()
        self.unit_menu = ttk.OptionMenu(self.unit, self.unit_var, 'MM', 'MM', 'Inch')
        self.unit_menu.grid(row=0, column=1)
        self.unit.pack()

        self.z_safe = LabelEntryFrame(frame_params={'master': self},
                                     label_params={'text': 'Z Safe'},
                                     entry_params={'width': 10},
                                     entry_default='10')
        
        self.x_interval = LabelEntryFrame(frame_params={'master': self},
                                     label_params={'text': 'X Interval'},
                                     entry_params={'width': 10},
                                     entry_default='20')
        
        self.y_interval = LabelEntryFrame(frame_params={'master': self},
                                     label_params={'text': 'Y Interval'},
                                     entry_params={'width': 10},
                                     entry_default='20')
        
        self.x_poinrts = LabelEntryFrame(frame_params={'master': self},
                                     label_params={'text': '#X Points'},
                                     entry_params={'width': 10},
                                     entry_default='3')
        
        self.y_poinrts = LabelEntryFrame(frame_params={'master': self},
                                     label_params={'text': '#Y Points'},
                                     entry_params={'width': 10},
                                     entry_default='3')
        
        self.preamble = LabelEntryFrame(frame_params={'master': self},
                                     label_params={'text': 'Preamble'},
                                     entry_params={'width': 35},
                                     entry_default='G17 G21 G90')
        
        self.postable = LabelEntryFrame(frame_params={'master': self},
                                     label_params={'text': 'Postamble'},
                                     entry_params={'width': 35},
                                     entry_default='M2')

        self.pack()

class GCodeFrame(ttk.LabelFrame):
    def __init__(self, listbox_select_callback, button_callback, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.frame = ttk.Frame(master=self)

        self.listbox = tkinter.Listbox(
            self.frame,
            # listvariable=self.lvar,
            height=6,
            selectmode=tkinter.SINGLE)

        self.listbox.grid(row=0, column=0)
        self.listbox.pack(side=tkinter.LEFT, fill=tkinter.BOTH)

        self.scrollbar = ttk.Scrollbar(
            master=self.frame,
            orient=tkinter.VERTICAL,
            command=self.listbox.yview
        )
        
        self.listbox['yscrollcommand'] = self.scrollbar.set

        # self.scrollbar.grid(row=0, column=1)
        self.scrollbar.pack(side=tkinter.LEFT, fill=tkinter.Y)

        self.listbox.bind('<<ListboxSelect>>', listbox_select_callback)
        self.listbox.configure(exportselection=False)

        self.frame.grid(row=0, column=0)

        self.gen_gcode = ttk.Button(self, text='Generate G-code', command=button_callback)
        self.gen_gcode.grid(row=1, column=0)
        
        self.pack()

class CommandFrame(ttk.LabelFrame):
    def __init__(self, button_callback, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.entry = ttk.Entry(master=self)
        self.entry.grid(row=0, column=0)

        self.run_command = ttk.Button(self, text='Run Command', command=button_callback)
        self.run_command.grid(row=1, column=0)
        
        self.pack()

class Application(tkinter.Tk):
    def __init__(self, open_builds_ctrl_addr, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.gantry_controller = OpenBuildsGantryController(open_builds_ctrl_addr)

        self.gantry_status = ttk.Label(master=self)
        self.gantry_status.pack()
        self.update_gantry_status()

        self.para_frame = ParaFrame(master=self, borderwidth=10, text='Parameters')
        # para_frame.pack()

        self.gcode_frame = GCodeFrame(listbox_select_callback=self.listbox_item_selected,
                                  button_callback=self.generate_gcode,
                                  master=self, borderwidth=10, text='G-code')
 
        self.command_frame = CommandFrame(button_callback=self.run_command, master=self, borderwidth=10, text='Command')

        self.generate_gcode()


    def listbox_item_selected(self, event):
        # get selected indices
        selected_indices = self.gcode_frame.listbox.curselection()
        if selected_indices:
            # print(selected_indices)
            # get selected items
            # selected_langs = ",".join([self.listbox.get(i) for i in selected_indices])
            # msg = f'You selected: {selected_langs}'
            # print(s)
            self.set_commmand_idx(selected_indices[0])
            # tkinter.messagebox.showinfo(title='Information', message=self.gcode[selected_indices[0]])
    
    def generate_gcode(self):
        self.gcode, self.command_idx = [], 0

        self.gcode.append(self.para_frame.preamble.entry.get() + '\n')
        
        x_pos, y_pos = [], []
        x_interval, y_interval = float(self.para_frame.x_interval.entry.get()), float(self.para_frame.y_interval.entry.get())
        x_points, y_points = int(self.para_frame.x_poinrts.entry.get()), int(self.para_frame.y_poinrts.entry.get())
        z_safe = float(self.para_frame.z_safe.entry.get())

        for i in range(x_points):
            x_pos.append(x_interval * i)

        for i in range(y_points):
            y_pos.append(y_interval * i)
        
        for x in x_pos:
            for y in y_pos:
                self.gcode.append(f'G00 Z{z_safe}' + '\n')
                self.gcode.append(f'G00 X{x} Y{y} Z0' + '\n')

        self.gcode.append(self.para_frame.postable.entry.get())

        # self.lvar.get()
        self.gcode_frame.listbox.delete(0, tkinter.END)
        for i, line in enumerate(self.gcode):
            self.gcode_frame.listbox.insert(tkinter.END, f'{i+1}. ' + line)
        
        self.set_command_entry()
        
        self.listbox_highlight_selection()
    
    def run_command(self):
        gcode_line = self.command_frame.entry.get()
        
        if not self.gantry_controller.gantry_ready():
            showinfo(title='Information', message='Gantry system is not ready.\nMake sure it is connected and in idle status.')
        else: 
            self.gantry_controller.run_one_line_gcode(gcode_line=gcode_line)

            self.set_commmand_idx(min(self.command_idx + 1, len(self.gcode) - 1))
            self.listbox_highlight_selection()
            print(gcode_line)
    
    def set_commmand_idx(self, idx):
            self.command_idx = idx
            self.set_command_entry()
            # self.command_frame.entry.delete(0, tkinter.END)
            # self.command_frame.entry.insert(0, self.gcode[self.command_idx])
    
    def set_command_entry(self):
        self.command_frame.entry.delete(0, tkinter.END)
        self.command_frame.entry.insert(0, self.gcode[self.command_idx])

    def listbox_highlight_selection(self):
        self.gcode_frame.listbox.select_clear(0, "end")
        self.gcode_frame.listbox.selection_set(self.command_idx)
        self.gcode_frame.listbox.see(self.command_idx)
        self.gcode_frame.listbox.activate(self.command_idx)
        self.gcode_frame.listbox.selection_anchor(self.command_idx)

    def update_gantry_status(self):
        self.gantry_status.config(text=f'Gantry status: {self.gantry_controller.gantry_status}')
        self.after(10, self.update_gantry_status)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Start the GUI.')
    parser.add_argument('open_builds_ctrl_addr', type=str, help='ip address of open builds control server')
    args = parser.parse_args()
    app = Application(args.open_builds_ctrl_addr)
    app.mainloop()
