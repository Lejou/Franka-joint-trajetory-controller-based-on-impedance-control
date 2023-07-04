#!/usr/bin/env python3

# import sys
# import copy
# import rospy
# from std_msgs.msg import Int16
# from kivymd.app import MDApp
# from kivy.lang import Builder
# from kivy.config import Config
# Config.set('kivy','keyboard_mode','systemanddock')
# from kivy.core.window import Window
# from kivymd.uix.screen import MDScreen

# class SimpleKV(MDApp):

# 	def __init__(self,**kwargs):
# 		super().__init__(**kwargs)
# 		self.screen = Builder.load_file('/home/lee/robot/src/th_demo/config/TestKivy.kv')
# 		self.slider_pub = rospy.Publisher("slider",Int16,queue_size=10)

# 	def build(self):
# 		return self.screen

# 	# def slider_function(self,slider_value):
# 	# 	# print(int(slider_value))
# 	# 	msg = Int16()
# 	# 	msg.data = int(slider_value)
# 	# 	self.slider_pub.publish(msg)
	

# if __name__ == '__main__':

# 	rospy.init_node('SimpleKV', anonymous=True)
# 	SimpleKV().run()


import kivy
from kivy.uix.boxlayout import BoxLayout
from kivy.core.window import Window
from kivy.lang import Builder
from kivy.event import ObjectWithUid
from kivy.metrics import dp
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.properties import StringProperty, ListProperty 
from kivy.utils import get_color_from_hex
from kivy.uix.widget import Widget

from kivymd.app import MDApp
from kivymd.uix.label import *
from kivymd.uix.button import MDFloatingActionButton
from kivymd.uix.textfield import MDTextField
from kivymd.uix.menu import MDDropdownMenu
from kivymd.uix.pickers import MDDatePicker, MDTimePicker
from kivymd.uix.snackbar import Snackbar
#from kivymd.uix.toolbar import toolbar

from DataBase import json_eidit




class Content(BoxLayout):
    text = StringProperty()
    Time = StringProperty()
    Date = StringProperty()


class Content_complet(BoxLayout):
    text = StringProperty()
    Time = StringProperty()
    Date = StringProperty()

class Separator(Widget):
    sep_color = ListProperty([0, 0, 0, 0.8])


class page_maniger(ScreenManager):
    pass

class Page_main(Screen):
    pass


class Page_secound(Screen):
    pass

class Page_therd(Screen):
    pass


class Example(MDApp):
    overlay_color = get_color_from_hex("#6042e4")
    
    def build(self):
        
        def start_slacting():
            self.menu.dismiss()
            self.root.ids.page_f.ids.main_list.selected_mode = True
            
        menu_items = [
            {
                "viewclass": "OneLineListItem",
                "text": "delete",
                "height": dp(40),
                "on_release": start_slacting,
            },
            {
                "viewclass": "OneLineListItem",
                "text": "Completed",
                "height": dp(40),
                "on_release": lambda : self.change_screen('Therd_page'),
            }

        ]
        self.menu = MDDropdownMenu(
            items=menu_items,
            width_mult=3,
        )
        kv = Builder.load_file('/home/lee/robot/src/th_demo/config/TestKivy.kv')
        return kv

    def on_start(self):
        self.f=[]
        self.t=[]
        if json_eidit.get_all_in("todo") != None:
            for i in json_eidit.get_all_in("todo"):
                self.root.ids.page_f.ids.main_list.add_widget(
                    Content(
                        text=i["name"],
                        Time=i["time"],
                        Date=i["date"],
                    )
                )
                self.f.append({"name":f'{i["name"]}',"time":f'{i["time"]}',"date":f'{i["date"]}'})
            instences = self.root.ids.page_f.ids.main_list.children
            for i in range(len(instences)):
                self.f[i]['instance'] = instences[-i-1]
        
        if json_eidit.get_all_in("complet_todo") != None:
            for i in json_eidit.get_all_in("complet_todo"):
                self.root.ids.page_t.ids.complet_main_list.add_widget(
                    Content_complet(
                        text=i["name"],
                        Time=i["time"],
                        Date=i["date"],
                    )
                )
                self.t.append({"name":f'{i["name"]}',"time":f'{i["time"]}',"date":f'{i["date"]}'})
            instences = self.root.ids.page_t.ids.complet_main_list.children
            for i in range(len(instences)):
                self.t[i]['instance'] = instences[-i-1]



    def on_checkbox_active(self, instance, states, i):
        data_checked = {'name':i.text,'time':i.Time,'date':i.Date}
        for y in self.f:
            t = {'name':y['name'],'time':y['time'],'date':y['date']}
            if data_checked == t:
                if t not in json_eidit.get_all_in('complet_todo'):
                    if t in json_eidit.get_all_in('todo'):
                            befor_ins_ins = self.root.ids.page_t.ids.complet_main_list.children
                            befor_ins = [str(i) for i in befor_ins_ins]
                            self.root.ids.page_t.ids.complet_main_list.add_widget(
                                Content_complet(
                                    text = t['name'],
                                    Time = t['time'],
                                    Date = t['date'],
                                )
                            )

                            json_eidit.add('complet_todo',t)
                            json_eidit.remove('todo',t)

                            after_ins = self.root.ids.page_t.ids.complet_main_list.children
                            if y in self.f:
                                self.f.remove(y)
                                self.root.ids.page_f.ids.main_list.remove_widget(y['instance'])
                                
                            y['instance']=after_ins[0]
                            self.t.append(y)


    def on_checkbox_deactive(self, instance, states, i):
        for u in self.t:
            data_checked = {'name':i.text,'time':i.Time,'date':i.Date}
            t = {'name':u['name'],'time':u['time'],'date':u['date']}
            if data_checked == t:
                if t not in json_eidit.get_all_in('todo'):
                    if t in json_eidit.get_all_in('complet_todo'):
                        befor_ins_ins = self.root.ids.page_t.ids.complet_main_list.children
                        befor_ins = [str(i) for i in befor_ins_ins]

                        self.root.ids.page_f.ids.main_list.add_widget(
                            Content(
                                text = t['name'],
                                Time = t['time'],
                                Date = t['date']
                            )
                        )

                    
                        json_eidit.add('todo',t)
                        json_eidit.remove('complet_todo',t)  

                        after_ins = self.root.ids.page_f.ids.main_list.children
                        if u in self.t:
                            self.t.remove(u)
                            self.root.ids.page_t.ids.complet_main_list.remove_widget(u['instance'])

                        u['instance']=after_ins[0]
                        self.f.append(u)
    

    def change_screen(self, page_nam):
        self.root.current = page_nam
        self.set_selection_mode('Jest to bypass',mode=False)

    def date_time_picker(self, focus_1_0):
        if focus_1_0:

            def get_date(instance, value, rainge):
                date_dialog.dismiss()
                date = str(value)
                self.root.ids.page_s.ids.Date_entry.text = date

                self.Time_entry = MDTextField(hint_text="Enter Time", text='')
                self.root.ids.page_s.ids.content_Layout.add_widget(self.Time_entry)
                time_dialog.open()

            def get_time(instance, value):
                date_dialog.dismiss()
                time = str(value)
                self.Time_entry.text = time

            date_dialog = MDDatePicker()
            date_dialog.bind(on_save=get_date, on_cancel=date_dialog.dismiss)

            time_dialog = MDTimePicker()
            time_dialog.bind(on_save=get_time, on_cancel=time_dialog.dismiss)

            date_dialog.open()

    def add_new_todo(self):
        todo_entry = str(self.root.ids.page_s.ids.Todo_entry.text)
        date_entry = str(self.root.ids.page_s.ids.Date_entry.text)

        if todo_entry != "" and date_entry != "":
            time_entry = str(self.Time_entry.text)
            self.root.ids.page_f.ids.main_list.add_widget(
                Content(
                    text=todo_entry,
                    Date=date_entry,
                    Time=time_entry,
                )
            )
            h = {'name':todo_entry,'date':date_entry,'time':time_entry}
            json_eidit.add("todo", h)
            self.root.current = "first_page"
            after_ins = self.root.ids.page_f.ids.main_list.children
            self.f.append({'name':todo_entry,'date':date_entry,'time':time_entry,'instance':after_ins[0]})

            self.root.ids.page_s.ids.Todo_entry.text = ''
            self.root.ids.page_s.ids.Date_entry.text = ''
            self.root.ids.page_s.ids.content_Layout.remove_widget(self.Time_entry)



        else:
            Snackbar(
                text="You did't enter task",
                snackbar_x="10dp",
                snackbar_y="10dp",
                size_hint_x=(Window.width - (dp(10) * 2)) / Window.width,
            ).open()
        

    
    def callback(self, button):
        self.menu.caller = button
        self.menu.open()

    def menu_callback(self, text_item):
        self.menu.dismiss()

    def upgrate_item(self, for_):
        todo = self.root.ids.page_s.ids.Todo_entry.text
        date = self.root.ids.page_s.ids.Date_entry.text
        time = self.Time_entry.text
        for_.children[1].text = todo
        for_.children[1].Date = date
        for_.children[1].Time = time

        for i in self.f:
            if i['instance'].children[1] == for_.children[1]:
                
                json_eidit.remove('todo', {'name':i['name'], 'time':i['time'], 'date':i['date']})
                i['name'] = todo
                i['date'] = date
                i['time'] = time
                json_eidit.add('todo', {'name':i['name'], 'time':i['time'], 'date':i['date']})
        self.root.ids.page_s.ids.Todo_entry.text = ''
        self.root.ids.page_s.ids.Date_entry.text = ''
        self.root.ids.page_s.ids.content_Layout.remove_widget(self.Time_entry)
        self.root.ids.page_s.remove_widget(self.Upgrate_buttom)
        self.change_screen('first_page')


    def eidit_todo(self, instance):
        for i in self.f:
            if i['instance'] == instance:
                self.Time_entry = MDTextField(hint_text="Enter Time", text=i['date'])
                self.change_screen('secound_page')
                self.root.ids.page_s.ids.Todo_entry.text = i['name']
                self.root.ids.page_s.ids.Date_entry.text = i['date']
                self.root.ids.page_s.ids.content_Layout.add_widget(self.Time_entry)
                self.Upgrate_buttom = MDFloatingActionButton(
                    icon = "circle-edit-outline",
                    md_bg_color = self.theme_cls.primary_color,
                    on_press = lambda x: self.upgrate_item(for_=instance),
                    pos_hint = {'center_x':0.7,'center_y':0.05}
                )
                self.root.ids.page_s.add_widget(self.Upgrate_buttom)
                

    def cancil_selection(self, obj):
        self.root.ids.page_f.ids.main_list.unselected_all()
        left_action_items = []
        right_action_items = [["dots-vertical", lambda x: self.callback(x)]]
        self.root.ids.page_f.ids.toolbar.title = "Inbox"

        self.root.ids.page_f.ids.toolbar.left_action_items = left_action_items
        self.root.ids.page_f.ids.toolbar.right_action_items = right_action_items



    def set_selection_mode(self, instance_selection_list, mode):
        g = self.root.ids.page_f.ids.main_list.get_selected_list_items()
        if len(g)!=0:
            if mode:
                left_action_items = [
                    ["close",lambda x:self.cancil_selection(x)]
                ]
                right_action_items = [
                    ["trash-can", self.deleat_selected_list_items],
                    ["select-all", lambda x: self.selected_all(x)],
                    ["dots-vertical", lambda x: self.callback(x)]
                ]
            else:
                left_action_items = []
                right_action_items = [["dots-vertical", lambda x: self.callback(x)]]
                self.root.ids.page_f.ids.toolbar.title = "Inbox"
            g = self.root.ids.page_f.ids.main_list.get_selected_list_items()
            if len(g) == 1:
                right_action_items = [
                    ["circle-edit-outline", lambda x: self.eidit_todo(g[0])],
                    ["trash-can", self.deleat_selected_list_items],
                    ["select-all", lambda x: self.selected_all(x)],
                    ["dots-vertical", lambda x: self.callback(x)]
                ]
            self.root.ids.page_f.ids.toolbar.left_action_items = left_action_items
            self.root.ids.page_f.ids.toolbar.right_action_items = right_action_items

    def selected_all(self,obj):
        self.root.ids.page_f.ids.main_list.selected_all()

    def on_selected(self, instance_selection_list, instance_selection_items):
        g = self.root.ids.page_f.ids.main_list.get_selected_list_items()
        if len(g)!=0:
            if len(g) == 1:
                right_action_items = [
                    ["circle-edit-outline", lambda x: self.eidit_todo(g[0])],
                    ["trash-can", self.deleat_selected_list_items], 
                    ["select-all", lambda x: self.selected_all(x)],
                    ["dots-vertical", lambda x: self.callback(x)]
                ]
                self.root.ids.page_f.ids.toolbar.right_action_items = right_action_items
            elif len(g) >= 1:
                right_action_items = [
                    ["trash-can", self.deleat_selected_list_items], 
                    ["select-all", lambda x: self.selected_all(x)],
                    ["dots-vertical", lambda x: self.callback(x)]
                ]
                self.root.ids.page_f.ids.toolbar.right_action_items = right_action_items
            self.root.ids.page_f.ids.toolbar.title = str(
                len(instance_selection_list.get_selected_list_items())
            )

    def on_unselected(self, instance_selection_list, instance_selection_item):
        g = self.root.ids.page_f.ids.main_list.get_selected_list_items()
        if instance_selection_list.get_selected_list_items():
            self.root.ids.page_f.ids.toolbar.title = str(
                len(instance_selection_list.get_selected_list_items())
            )
        if len(g) == 1:
            right_action_items = [
                ["circle-edit-outline", lambda x: self.eidit_todo(g[0])],
                ["trash-can", self.deleat_selected_list_items], 
                ["select-all", lambda x: self.selected_all(x)],
                ["dots-vertical", lambda x: self.callback(x)]
            ]
            self.root.ids.page_f.ids.toolbar.right_action_items = right_action_items
        if len(g) == 0:
            left_action_items = []
            right_action_items = [["dots-vertical", lambda x: self.callback(x)]]
            self.root.ids.page_f.ids.toolbar.title = "Inbox"

            self.root.ids.page_f.ids.toolbar.left_action_items = left_action_items
            self.root.ids.page_f.ids.toolbar.right_action_items = right_action_items

    def deleat_selected_list_items(self, instance):
        g = self.root.ids.page_f.ids.main_list.get_selected_list_items()
        for i in g:
            self.root.ids.page_f.ids.main_list.remove_widget(i)
        self.root.ids.page_f.ids.toolbar.title = "Select to Delete"
        item_after = self.root.ids.page_f.ids.main_list.children
        end = []
        for y in self.f:
            if y['instance'] not in item_after:
                del y['instance']
                json_eidit.remove('todo',y)
            elif y['instance'] in item_after:
                end.append(y)
        self.f = end
        


Example().run()
