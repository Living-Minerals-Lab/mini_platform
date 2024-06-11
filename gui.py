from ast import main
import tkinter
from tkinter import ttk
import tkinter.messagebox


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
    def __init__(self, list_box_select_callback, button_callback, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.list_box = tkinter.Listbox(
            self,
            # listvariable=self.lvar,
            height=6,
            selectmode=tkinter.SINGLE)

        self.list_box.grid(row=0, column=0)

        self.scrollbar = ttk.Scrollbar(
            self,
            orient=tkinter.VERTICAL,
            command=self.list_box.yview
        )
        
        self.list_box['yscrollcommand'] = self.scrollbar.set

        self.scrollbar.grid(row=0, column=1)

        self.list_box.bind('<<ListboxSelect>>', list_box_select_callback)
        self.list_box.configure(exportselection=False)

        self.gen_g_code = ttk.Button(self, text='Generate G-code', command=button_callback)
        self.gen_g_code.grid(row=1, column=0)
        
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
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.para_frame = ParaFrame(master=self, borderwidth=10, text='Parameters')
        # para_frame.pack()

        self.g_code_frame = GCodeFrame(list_box_select_callback=self.items_selected,
                                  button_callback=self.generate_g_code,
                                  master=self, borderwidth=10, text='G-code')
        # ttk.LabelFrame(master=self, borderwidth=10, text='G-code')
        # g_code_frame.pack()
        self.command_frame = CommandFrame(self.run_command, master=self, borderwidth=10, text='Command')

        
    def items_selected(self, event):
        # get selected indices
        selected_indices = self.g_code_frame.list_box.curselection()
        if selected_indices:
            print(selected_indices)
            # get selected items
            # selected_langs = ",".join([self.list_box.get(i) for i in selected_indices])
            # msg = f'You selected: {selected_langs}'
            # print(s)
            self.command_idx = selected_indices[0]
            tkinter.messagebox.showinfo(title='Information', message=self.gcode[selected_indices[0]])
    
    def generate_g_code(self):
        print('generate g-code')
    
    def run_command(self):
        print('run command')


if __name__ == '__main__':
    app = Application()
    app.mainloop()
