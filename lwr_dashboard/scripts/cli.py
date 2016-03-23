#!/usr/bin/env python
# -*- coding: utf-8 -*-
#title           :menu.py
#description     :This program displays an interactive menu on CLI
#author          :https://www.bggofurther.com/2015/01/create-an-interactive-command-line-menu-using-python/
 
import sys, os
from lwr_dashboard.lwr_dashboard import LwrDashboard
import rospy

# Main definition - constants
menu_actions  = {}  

# =======================
#     MENUS FUNCTIONS
# =======================
 
# Main menu
def main_menu():
    os.system('clear')
    
    print "Welcome,\n"
    print "Please choose the menu you want to start:"
    print "1. Control menu"
    print "2. Menu 2"
    print "\n0. Quit"
    choice = raw_input(" >>  ")
    exec_menu(choice)
 
    return
 
# Execute menu
def exec_menu(choice):
    os.system('clear')
    ch = choice.lower()
    if ch == '':
        menu_actions['main_menu']()
    else:
        try:
            menu_actions[ch]()
        except KeyError:
            print "Invalid selection, please try again.\n"
            menu_actions['main_menu']()
    return
 
# Menu 1
def menu1():
    #os.system('clear')
    print "Control Menu !\n"
    print "3. FRISTART"
    print "4. FRISTOP"
    print "5. ENABLE Motors"
    print "6. DISABLE Motors"
    print "7. MOVE START POS"
    print "9. Back"
    print "0. Quit"
    choice = raw_input(" >>  ")
    exec_menu(choice)
    return
 
 
# Menu 2
def menu2():
    #os.system('clear')
    print "Hello Menu 2 !\n"
    print "9. Back"
    print "0. Quit" 
    choice = raw_input(" >>  ")
    exec_menu(choice)
    return

def fristart():
    print "fristart"
    lwrdb.command_mode_request()
    menu1()
        
def fristop():
    print "fristop"
    lwrdb.monitor_mode_request()
    menu1()

def enable_motors():
    print "fristart"
    lwrdb.enable_motors()
    menu1()
    
def disable_motors():
    print "fristart"
    lwrdb.enable_motors()
    menu1()

def move_start_pos():
    print "fristart"
    lwrdb.move_start_pos()
    menu1()


# Back to main menu
def back():
    menu_actions['main_menu']()
 
# Exit program
def exit():
    sys.exit()
 
# =======================
#    MENUS DEFINITIONS
# =======================
 
# Menu definition
menu_actions = {
    'main_menu': main_menu,
    '1': menu1,
    '2': menu2,
    '3': fristart,
    '4': fristop,
    '5': enable_motors,
    '6': disable_motors,
    '7': move_start_pos,
    '9': back,
    '0': exit,
}
 
# =======================
#      MAIN PROGRAM
# =======================
 
# Main Program
if __name__ == "__main__":
    rospy.init_node('lwr_dashboard')
    lwrdb = LwrDashboard(namespace="ra")
    # Launch main menu
    main_menu()
