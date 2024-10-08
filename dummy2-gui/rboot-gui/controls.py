from datetime import datetime
from typing import Any

from nicegui import ui
from odrive.pyfibre import fibre
from odrive.utils import dump_errors
import can_data
import struct
import time

default = {'motor': 0}
motors_cfg = {'M1': 
                {'position': 0.0, 
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 50,
                'teaching': 0,
                'ccw': 0,
                },
                'M2':
                {'position': 0.0,
                'velocity': 0.0,                   
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 50,
                'teaching': 0,
                'ccw': 0,
                },
                'M3':
                {'position': 0.0,
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 50,
                'teaching': 0,
                'ccw': 0,
                },
                'M4':
                {'position': 0.0,
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 30,
                'teaching': 0,
                'ccw': 0,
                },
                'M5':
                {'position': 0.0,
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 50,
                'teaching': 0,
                'ccw': 0,
                },
                'M6':
                {'position': 0.0,
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 30,
                'teaching': 0,
                'ccw': 0,
                },
                }

MODES = {
    0: 'Status',
    1: 'Controls',
    2: 'Teaching',
}

teaching = []    
count = 0    

def controls(client) -> None:
    def get_reduction(i):
        motor_keys = list(motors_cfg.keys()) 
        motor_name = motor_keys[i]
        return motors_cfg[motor_name]['reduction']
         
    def send_msg(id, type, cmd1, cmd2) -> None:
         client.send_message(id, type, struct.pack('<I', cmd1), struct.pack('<I', cmd2), can_data.Message_type['short'])

    def send_6d_msg(id, type, cmd1, cmd2) -> None:
        for i in range(6):
            cid = i + 1
            client.send_message(cid, type, struct.pack('<I', cmd1), struct.pack('<I', cmd2), can_data.Message_type['short'])

    def send_position(id, sign: int, position) -> None:
        cid = int(id)
        reduction_value = get_reduction(cid - 1)
        motorCnt = position / 360.0 * reduction_value
        pos = struct.pack('<f', sign * float(motorCnt))
        cmd2 = struct.pack('<HH', 60, 10)
        print(cid, sign, position, pos, cmd2)
        client.send_message(cid, can_data.command_id['Set_Input_Pos'], pos, cmd2, can_data.Message_type['short'])

    def send_6d_position(sign: int, position) -> None:
        for i, a in enumerate(position):
            cid = i + 1
            # if i == 3
            reduction_value = get_reduction(i)
            motorCnt = a / 360.0 * reduction_value
            pos = struct.pack('<f', sign * float(motorCnt))
            cmd2 = struct.pack('<HH', 0x1f, 0)
            client.send_message(cid, can_data.command_id['Set_Input_Pos'], pos, cmd2, can_data.Message_type['short'])
            #give an break to make sure udp package sent out success
            time.sleep(0.01)

    def udp_callback(data):
         update(data)
    
    def register_cb():
        client.register_callback(udp_callback) 
        send_msg(1, can_data.command_id['Set_Axis_State'], can_data.AxisState['IDLE'], can_data.Message_type['short'])        

    def unregister_cb():
        client.unregister_callback() 
        buff_status.set_text(f'CAN BUS: Not enabled')
        buff_status.style('color: #fc0320; font-weight: bold')

    # with ui.row().classes('w-full justify-between items-center'):
    #     with ui.row():
    #         ui.button('Start', on_click=register_cb)

    with ui.row().classes('w-full justify-between items-center'):
        with ui.row():
            buff_status = ui.label(f'CAN BUS:')

        with ui.row():
            ui.button(on_click=register_cb).props('icon=radio_button_checked round') \
                                    .tooltip('Connect to CAN BUS')
            ui.button(on_click=unregister_cb).props('icon=cancel round') \
                                    .tooltip('Disconnect to CAN BUS')
            ui.button(on_click=lambda: send_6d_msg(0, can_data.command_id['Set_Axis_State'], can_data.AxisState['CLOSED_LOOP_CONTROL'], 0)) \
                .props('icon=repeat round') \
                .tooltip('Enable all joints to close loop mode')
            ui.button(on_click=lambda: send_6d_msg(0, can_data.command_id['Set_Axis_State'], can_data.AxisState['IDLE'], 0)).props('icon=close round') \
                .tooltip('Enable all joints to idle mode')

        # with ui.row():
        #     ui.button(on_click=lambda: client.send_message("enable")) \
        #         .props('icon=restart_alt flat round') \
        #         .tooltip('enable monitor of can bus')

    # let's set toggled motor
    with ui.row():
        mode = ui.toggle(MODES).bind_value(default, 'motor')

    with ui.row():
        count = 0
        # forw_data = [0, 0.0]
        for k,v in motors_cfg.items():
            count += 1
            # forw_data[0] += 1
            with ui.card().bind_visibility_from(mode, 'value', value=0):
                ui.markdown(f'##### {k}')
                with ui.column():
                    with ui.row():
                            ui.label('Status:')
                            ui.label('Status').bind_text_from(v, 'status')
                            ui.label('Error:')
                            ui.label('Error').bind_text_from(v, 'error')
                            ui.label('V:')
                            ui.label('V').bind_text_from(v, 'voltage')
                            # ui.label('Iq:')
                            # ui.label('Iq').bind_text_from(v, 'iq')
                            # ui.label('T:')
                            # ui.label('T').bind_text_from(v, 'ibus')
                    with ui.row():
                            ui.number('position', format='%.3f').bind_value(v, 'position').set_enabled(False)
                            ui.number('velocity', format='%.3f').bind_value(v, 'velocity').set_enabled(False)
                    with ui.row():
                            ui.number('Iq', format='%.3f').bind_value(v, 'iq').set_enabled(False)
                            ui.number('Ibus', format='%.3f').bind_value(v, 'ibus').set_enabled(False)
                    with ui.column():
                        with ui.row().classes('w-full'):
                                ui.button(on_click=lambda count=count: send_msg(count, can_data.command_id['Set_Axis_State'], can_data.AxisState['CLOSED_LOOP_CONTROL'], 0)) \
                                    .props('icon=repeat round') \
                                    .tooltip('Enable close loop mode')
                                ui.button(on_click=lambda count=count: send_msg(count, can_data.command_id['Set_Axis_State'], can_data.AxisState['IDLE'], 0)) \
                                    .props('icon=close round') \
                                    .tooltip('Enable idle mode')
                                ui.button(on_click=lambda count=count: send_msg(count, can_data.command_id['Reboot'], can_data.Reboot['Reboot'], 0)) \
                                    .props('icon=restart_alt round') \
                                    .tooltip('Reboot motor')
                # m1_pos = ui.label().bind_text_from(v, 'id')
    with ui.row():
        with ui.card().bind_visibility_from(mode, 'value', value=1):
            ui.markdown(f'##### Joint Control')
            with ui.row():
                with ui.column():
                    can_id = ui.number('Joint id', value=0)
                with ui.column():
                    position = ui.number('Input position', value=0)
                    def send_position_l(id, loc): send_position(id, loc, position.value)
            with ui.row():
                    ui.button(on_click=lambda: send_position_l(can_id.value, -1)).props('round flat icon=skip_previous')
                    ui.button(on_click=lambda: send_position_l(can_id.value, 0)).props('round flat icon=exposure_zero')
                    ui.button(on_click=lambda: send_position_l(can_id.value, 1)).props('round flat icon=skip_next')
            
        with ui.card().bind_visibility_from(mode, 'value', value=1):
            ui.markdown(f'##### Rboot Arm Control')
            with ui.row():
                with ui.column():
                    j1 = ui.number('J1', value=0)
                with ui.column():
                    j2 = ui.number('J2', value=0)
                with ui.column():
                    j3 = ui.number('J3', value=0)
                with ui.column():
                    j4 = ui.number('J4', value=0)
                with ui.column():
                    j5 = ui.number('J5', value=0)
                with ui.column():
                    j6 = ui.number('J6', value=0)
            with ui.row():
                    ui.button("Home", on_click=lambda: send_6d_position(0, [j1.value, j2.value, j3.value, j4.value, j5.value, j6.value])).props('round flat')
                    ui.button("Send", on_click=lambda: send_6d_position(1, [j1.value, j2.value, j3.value, j4.value, j5.value, j6.value])).props('round flat')

        with ui.card().bind_visibility_from(mode, 'value', value=2):
            ui.markdown(f'##### Rboot teaching controls')
            def robot_teaching_control(e):
                 motors_cfg['M5']['teaching'] = e

            with ui.row():
                    ui.button("Start", on_click=lambda: robot_teaching_control(1)).props('round flat')
                    ui.button("Stop", on_click=lambda: robot_teaching_control(0)).props('round flat')
                    ui.button("Replay", on_click=lambda: robot_teaching()).props('round flat')

    def robot_teaching():
        for i in range(len(teaching)):
            send_position(5, 1, teaching[i])
            print(teaching)

    def update(data):
        #data = client.get_buffer_data()
        if data is None:
            buff_status.set_text(f'CAN BUS: Not enabled')
            buff_status.style('color: #fc0320; font-weight: bold')
        else:
            buff_status.set_text(f'CAN BUS: Enabled')
            buff_status.style('color: #03fc1c; font-weight: bold')
            # Split the string by spaces and convert each hexadecimal value to an integer
            int_values = [int(x, 16) for x in data.split()]
            byte_array = struct.pack('12B', *int_values)
            # print(int_values, byte_array)
            if len(byte_array) == 12:
                msg = can_data.pack_can_message(byte_array)
                # print(msg)
                count = 0
                id = msg.get('id')
                if id >= 0x30: print('CAN BUS ID MUST less than 48(0x30)!!!')
                for k,v in motors_cfg.items():
                    count += 1
                    if k == 'M'+str(count) and count == id:
                        type = msg.get('type')
                        body = msg.get('body')
                        if type == can_data.command_id['Get_Encoder_Estimates']:
                            pos, vel = struct.unpack('<ff', body)
                            v['position'] = pos
                            v['velocity'] = vel
                            if id == 5 and v['teaching'] == 1: 
                                 teaching.append(pos)
                                #  print(v['teaching'])
                        elif type == can_data.command_id['Heartbeat']:
                            error, state, result, traj_done, reserved = struct.unpack('<IBBBB', body)
                            if result == 0:
                                v['status'] = state
                                v['error'] = error
                        elif type == can_data.command_id['Get_Bus_Voltage_Current']:
                             vol, iq = struct.unpack('<ff', body)
                             v['voltage'] = round(vol, 2)
                             v['ibus'] = iq
                        elif type == can_data.command_id['Get_Iq']:
                             iqs, iq = struct.unpack('<ff', body)
                            #  v['ibus'] = iqs
                             v['iq'] = iq 
                        elif type == can_data.command_id['Get_Temperature']:
                             f, m = struct.unpack('<ff', body)
                             print(f, m)
    # ui.timer(0.01, update)
    # ui.timer.cancel

