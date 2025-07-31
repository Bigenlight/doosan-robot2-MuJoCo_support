import rclpy
import os
import sys
import time

from rclpy.logging import get_logger

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

logger = get_logger('single_robot_simple_logger')

def main(args=None):
        rclpy.init(args=args)

        node = rclpy.create_node('single_robot_simple_py', namespace=ROBOT_ID)

        DR_init.__dsr__node = node

        try:
                from DSR_ROBOT2 import print_ext_result, movej, movejx, movesj, movesx, movel, movec, move_periodic, move_spiral, moveb, set_velx, set_accx, set_robot_mode
                from DSR_ROBOT2 import posj, posx, posb, ROBOT_MODE_MANUAL
                from DSR_ROBOT2 import DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS, ROBOT_MODE_AUTONOMOUS
                # Import tool management functions
                from DSR_ROBOT2 import add_tool, del_tool, get_tool, set_tool
                # print_result("Import DSR_ROBOT2 Success!")
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return

        r = set_robot_mode(ROBOT_MODE_MANUAL)
        print(f"Set robot mode: {r}")
        time.sleep(0.5)
        
        # Tool management testing
        print("=== Tool Management Testing ===")
        
        # Test 1: Create a new tool
        tool_name = "tool#1"
        tool_weight = 2.5
        tool_cog = [10.0, 10.0, 10.0]
        tool_inertia = [0.1, 0.1, 0.1, 0.0, 0.0, 0.0] 

        print(f"Creating tool: {tool_name}")
        result = add_tool(tool_name, tool_weight, tool_cog, tool_inertia)
        if result == 0:
                print(f"Tool '{tool_name}' created successfully")
        else:
                print(f"Failed to create tool '{tool_name}'")
        
        # Test 2: Get current tool (should be empty initially)
        current_tool = get_tool()
        print(f"Current tool before setting: '{current_tool}'")

        # Test 3: Set the created tool as current tool
        print(f"Setting tool: {tool_name}")
        result = set_tool(tool_name)
        if result == 0:
                print(f"Tool '{tool_name}' set successfully")
        else:
                print(f"Failed to set tool '{tool_name}'")
           
        # Test 4: Get current tool again (should show our tool)
        current_tool = get_tool()
        print(f"Current tool after setting: '{current_tool}'")

        r = set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print(f"Set robot mode: {r}")

        # Add transition time as mentioned in the documentation
        time.sleep(1)

        p1= posj(0,0,0,0,0,0)                    #joint
        print(p1)
        p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0) #joint

        x1= posx(400, 500, 800.0, 0.0, 180.0, 0.0) #task
        x2= posx(400, 500, 500.0, 0.0, 180.0, 0.0) #task

        c1 = posx(559,434.5,651.5,0,180,0)
        c2 = posx(559,434.5,251.5,0,180,0)


        q0 = posj(0,0,0,0,0,0)
        q1 = posj(10, -10, 20, -30, 10, 20)
        q2 = posj(25, 0, 10, -50, 20, 40) 
        q3 = posj(50, 50, 50, 50, 50, 50) 
        q4 = posj(30, 10, 30, -20, 10, 60)
        q5 = posj(20, 20, 40, 20, 0, 90)
        qlist = [q0, q1, q2, q3, q4, q5]

        x1 = posx(600, 600, 600, 0, 175, 0)
        x2 = posx(600, 750, 600, 0, 175, 0)
        x3 = posx(150, 600, 450, 0, 175, 0)
        x4 = posx(-300, 300, 300, 0, 175, 0)
        x5 = posx(-200, 700, 500, 0, 175, 0)
        x6 = posx(600, 600, 400, 0, 175, 0)
        xlist = [x1, x2, x3, x4, x5, x6]


        X1 =  posx(370, 670, 650, 0, 180, 0)
        X1a = posx(370, 670, 400, 0, 180, 0)
        X1a2= posx(370, 545, 400, 0, 180, 0)
        X1b = posx(370, 595, 400, 0, 180, 0)
        X1b2= posx(370, 670, 400, 0, 180, 0)
        X1c = posx(370, 420, 150, 0, 180, 0)
        X1c2= posx(370, 545, 150, 0, 180, 0)
        X1d = posx(370, 670, 275, 0, 180, 0)
        X1d2= posx(370, 795, 150, 0, 180, 0)


        seg11 = posb(DR_LINE, X1, radius=20)
        seg12 = posb(DR_CIRCLE, X1a, X1a2, radius=21)
        seg14 = posb(DR_LINE, X1b2, radius=20)
        seg15 = posb(DR_CIRCLE, X1c, X1c2, radius=22)
        seg16 = posb(DR_CIRCLE, X1d, X1d2, radius=23)
        b_list1 = [seg11, seg12, seg14, seg15, seg16]

        # Main robot movement loop
        loop_count = 0
        max_loops = 1
        
        while rclpy.ok() and loop_count < max_loops:
                print(f"=== Movement Loop {loop_count + 1} ===")
                
                # Check current tool before movement
                current_tool = get_tool()
                print(f"Current tool during movement: '{current_tool}'")
                
                movej(p2, vel=100, acc=100)
                time.sleep(1)
                
                movej(p1, vel=100, acc=100)
                time.sleep(1)
                
                loop_count += 1

        # Test 5: Clean up - Reset tool to empty and delete the created tool
        print("=== Cleaning up tools ===")
        r = set_robot_mode(ROBOT_MODE_MANUAL)
        print(f"Set robot mode: {r}")

        # Reset current tool to empty
        print("Resetting current tool to empty")
        result = set_tool("")
        if result == 0:
                print("Current tool reset successfully")
        else:
                print("Failed to reset current tool")
        
        time.sleep(1)
        
        # Verify current tool is empty
        current_tool = get_tool()
        print(f"Current tool after reset: '{current_tool}'")
        
        # Delete the created tool
        print(f"Deleting tool: {tool_name}")
        result = del_tool(tool_name)
        if result == 0:
                print(f"Tool '{tool_name}' deleted successfully")
        else:
                print(f"Failed to delete tool '{tool_name}'")

        print('good bye!')
        rclpy.shutdown()

if __name__ == "__main__":
        main()
