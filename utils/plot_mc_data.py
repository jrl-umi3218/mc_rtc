#!/usr/bin/env python

import csv
import itertools
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np

plt.ion()

import sys

REF_JOINT_ORDER = [
    "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
    "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5",
    "CHEST_JOINT0", "CHEST_JOINT1", "HEAD_JOINT0", "HEAD_JOINT1",
    "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6", "RARM_JOINT7",
    "LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6", "LARM_JOINT7",
    "RHAND_JOINT0", "RHAND_JOINT1", "RHAND_JOINT2", "RHAND_JOINT3", "RHAND_JOINT4",
    "LHAND_JOINT0", "LHAND_JOINT1", "LHAND_JOINT2", "LHAND_JOINT3", "LHAND_JOINT4"]

def usage():
    print "{0} [.log]".format(sys.argv[0])

if len(sys.argv) < 2:
    usage()
    sys.exit(0)

data = {}
with open(sys.argv[1]) as fd:
    reader = csv.DictReader(fd, delimiter=';')
    for row in reader:
        for k in reader.fieldnames:
            try:
                value = float(row[k])
            except ValueError:
                value = None
            if k in data:
                data[k].append(value)
            else:
                data[k] = [value]
    for k in data:
        data[k] = np.array(data[k])
    print "Read data from log file"

if not len(data):
    print "Failed to read data from provided file"
    sys.exit(0)

assert('t' in data)
data['t'] = data['t'] - data['t'][0]

def set_ylim(ax, ymin, ymax):
    if ymin < 0:
        ymin = 1.2*ymin
    else:
        ymin = 0.8*ymin
    if ymax > 0:
        ymax = 1.2*ymax
    else:
        ymax = 0.8*ymax
    if ymax > abs(ymin):
        ymin = -ymax
    else:
        ymax = -ymin
    ax.set_xlim(xmin = 0, xmax = np.max(data['t']))
    ax.set_ylim(ymin = ymin, ymax = ymax)

def get_ylim(ax):
    ymin, ymax = ax.get_ylim()
    if ymax == 1.0:
        ymax = -1e6
    if ymin == 0.0:
        ymin = 1e6
    return ymin, ymax

def plot_stance_index_fig(ax, scale, cc):
    if 'stance_index' not in data:
      return
    if scale:
        ymin, ymax = ax.get_ylim()
        ymin += 0.1*(ymax-ymin)
        ymax -= 0.1*(ymax-ymin)
        nmax = np.max(data['stance_index'])
        ax.plot(data['t'], data['stance_index']*(ymax-ymin)/nmax + ymin, color = cc.next(), alpha = 0.5)
    else:
        ax.plot(data['t'], data['stance_index'], color = cc.next(), alpha = 0.5)
        ax.set_ylabel('stance')

def plot_torque_fig(joint_names, ax, cc):
    ymin, ymax = get_ylim(ax)
    for j in joint_names:
        if j in REF_JOINT_ORDER:
            tauc_idx = 'tauIn_' + str(REF_JOINT_ORDER.index(j))
            if tauc_idx in data:
                ax.plot(data['t'], data[tauc_idx], label='{0} torque'.format(j), color = cc.next())
                ymin = min(ymin, np.min(data[tauc_idx]))
                ymax = max(ymax, np.max(data[tauc_idx]))
    set_ylim(ax, ymin, ymax)
    ax.set_ylabel('Torque')

def plot_encoder_fig(joint_names, ax, cc):
    ymin, ymax = get_ylim(ax)
    for j in joint_names:
        if j in REF_JOINT_ORDER:
            qIn_idx = 'qIn_' + str(REF_JOINT_ORDER.index(j))
            if qIn_idx in data:
                ax.plot(data['t'], data[qIn_idx], label='{0} encoder'.format(j), color = cc.next())
                ymin = min(ymin, np.min(data[qIn_idx]))
                ymax = max(ymax, np.max(data[qIn_idx]))
    set_ylim(ax, ymin, ymax)
    ax.set_ylabel('Encoder')

def plot_command_fig(joint_names, ax, cc):
    ymin, ymax = get_ylim(ax)
    for j in joint_names:
        if j in REF_JOINT_ORDER:
            qOut_idx = 'qOut_' + str(REF_JOINT_ORDER.index(j))
            if qOut_idx in data:
                ax.plot(data['t'], data[qOut_idx], label='{0} command'.format(j), color = cc.next())
                ymin = min(ymin, np.min(data[qOut_idx]))
                ymax = max(ymax, np.max(data[qOut_idx]))
    set_ylim(ax, ymin, ymax)
    ax.set_ylabel('Command')

def plot_velocity_command_fig(joint_names, ax, cc):
    ymin, ymax = get_ylim(ax)
    for j in joint_names:
        if j in REF_JOINT_ORDER:
            qOut_idx = 'qOut' + str(REF_JOINT_ORDER.index(j))
            if qOut_idx in data:
                vOut = [0]
                vOut.extend([(180/np.pi)*(data[qOut_idx][i+1] - data[qOut_idx][i])/0.005 for i in xrange(len(data[qOut_idx])-1)])
                ax.plot(data['t'], vOut, label='{0} velocity command'.format(j), color = cc.next())
                ymin = min(ymin, np.min(vOut))
                ymax = max(ymax, np.max(vOut))
    set_ylim(ax, ymin, ymax)
    ax.set_ylabel('Velocity command')

def plot_velocity_actual_fig(joint_names, ax, cc):
    ymin, ymax = get_ylim(ax)
    for j in joint_names:
        if j in REF_JOINT_ORDER:
            qIn_idx = 'qIn_' + str(REF_JOINT_ORDER.index(j))
            if qIn_idx in data:
                vIn = [0]
                vIn.extend([(180/np.pi)*(data[qIn_idx][i+1] - data[qIn_idx][i])/0.005 for i in xrange(len(data[qIn_idx])-1)])
                ax.plot(data['t'], vIn, label='{0} velocity motor'.format(j), color = cc.next())
                ymin = min(ymin, np.min(vIn))
                ymax = max(ymax, np.max(vIn))
    set_ylim(ax, ymin, ymax)
    ax.set_ylabel('Velocity motors')

def plot_error_fig(joint_names, ax, cc):
    ymin, ymax = get_ylim(ax)
    for j in joint_names:
        if j in REF_JOINT_ORDER:
            qIn_idx = 'qIn_' + str(REF_JOINT_ORDER.index(j))
            qOut_idx = 'qOut_' + str(REF_JOINT_ORDER.index(j))
            if qIn_idx in data and qOut_idx in data:
                y_data = data[qOut_idx] - data[qIn_idx]
                ax.plot(data['t'], y_data, label='{0} error'.format(j), color = cc.next())
                ymin = min(ymin, np.min(y_data))
                ymax = max(ymax, np.max(y_data))
    set_ylim(ax, ymin, ymax)
    ax.set_ylabel('Error')

def plot_force_fig(force_sensors, ax, cc):
    ymin, ymax = get_ylim(ax)
    for force_sensor in force_sensors:
        if force_sensor+'ForceSensor_fx' in data:
            for i in ['fx', 'fy', 'fz']:
                fv = force_sensor + 'ForceSensor_' + i
                ax.plot(data['t'], data[fv], label = '{0}: {1}'.format(force_sensor, i), color = cc.next())
                ymin = min(ymin, np.min(data[fv]))
                ymax = max(ymax, np.max(data[fv]))
            set_ylim(ax, ymin, ymax)
            ax.set_ylabel('Force')

def plot_moment_fig(force_sensors, ax, cc):
    ymin, ymax = get_ylim(ax)
    for force_sensor in force_sensors:
        if force_sensor+'ForceSensor_cx' in data:
            for i in ['cx', 'cy', 'cz']:
                fv = force_sensor + 'ForceSensor_' + i
                ax.plot(data['t'], data[fv], label = '{0}: {1}'.format(force_sensor, i), color = cc.next())
                ymin = min(ymin, np.min(data[fv]))
                ymax = max(ymax, np.max(data[fv]))
            set_ylim(ax, ymin, ymax)
            ax.set_ylabel('Moment')

def plot_gen3d_fig(name, ax, cc):
    ymin, ymax = get_ylim(ax)
    if name + 'x' in data:
        for i in ['x', 'y', 'z']:
            v = name + i
            ax.plot(data['t'], data[v], label = '{0}: {1}'.format(name, i), color = cc.next())
            ymin = min(ymin, np.min(data[v]))
            ymax = max(ymax, np.max(data[v]))
        set_ylim(ax, ymin, ymax)
        ax.set_ylabel(name + ' SI')

def plot_rpy_fig(ax, cc):
    ymin, ymax = get_ylim(ax)
    if 'rpy_r' in data:
        for i in ['r', 'p', 'y']:
            v = 'rpy_' + i
            ax.plot(data['t'], data[v], label = '{0}: {1}'.format('rpy', i), color = cc.next())
            ymin = min(ymin, np.min(data[v]))
            ymax = max(ymax, np.max(data[v]))
        set_ylim(ax, ymin, ymax)
        ax.set_ylabel(name + ' SI')

def plot_gen4d_fig(name, ax, cc):
    ymin, ymax = get_ylim(ax)
    if name + 'w' in data:
        for i in ['w', 'x', 'y', 'z']:
            v = name + i
            ax.plot(data['t'], data[v], label = '{0}: {1}'.format(name, i), color = cc.next())
            ymin = min(ymin, np.min(data[v]))
            ymax = max(ymax, np.max(data[v]))
        set_ylim(ax, ymin, ymax)
        ax.set_ylabel(name + ' SI')

def prep_ax(title):
    fig, ax = plt.subplots()
    fig.canvas.set_window_title(title)
    color_cycler = itertools.cycle(['r','g','b','y','k','c','m','orange'])
    ax2 = ax.twinx()
    return ax,ax2,color_cycler

def sanitize_args(joint_names):
    if isinstance(joint_names, str):
        return [joint_names]
    else:
        return joint_names

def plot_torque(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Torque: ' + ','.join(joint_names))
    plot_torque_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_stance_index_fig(ax2, False, cc)
    plt.show()

def plot_encoder(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Encoder: ' + ','.join(joint_names))
    plot_encoder_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_stance_index_fig(ax2, False, cc)
    plt.show()

def plot_command(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Command: ' + ','.join(joint_names))
    plot_command_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_stance_index_fig(ax2, False, cc)
    plt.show()

def plot_velocity_command(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Velocity command: ' + ','.join(joint_names))
    plot_velocity_command_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_stance_index_fig(ax2, False, cc)
    plt.show()

def plot_velocity_actual(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Velocity actual: ' + ','.join(joint_names))
    plot_velocity_actual_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_stance_index_fig(ax2, False, cc)
    plt.show()

def plot_error(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Error: ' + ','.join(joint_names))
    plot_error_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_stance_index_fig(ax2, False, cc)
    plt.show()

def plot_torque_error(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Torque/Error: ' + ','.join(joint_names))
    plot_torque_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_error_fig(joint_names, ax2, cc)
    plot_stance_index_fig(ax2, True, cc)
    ax2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plt.show()

def plot_command_encoder(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Command/Encoder: ' + ','.join(joint_names))
    plot_command_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_encoder_fig(joint_names, ax2, cc)
    plot_stance_index_fig(ax2, True, cc)
    ax2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plt.show()

def plot_velocity_command_actual(joint_names):
    joint_names = sanitize_args(joint_names)
    ax, ax2, cc = prep_ax('Command/Encoder: ' + ','.join(joint_names))
    plot_velocity_command_fig(joint_names, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_velocity_actual_fig(joint_names, ax2, cc)
    plot_stance_index_fig(ax2, True, cc)
    ax2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plt.show()

def plot_force(force_sensors):
    force_sensors = sanitize_args(force_sensors)
    ax, ax2, cc = prep_ax('Force/Moment: ' + ','.join(force_sensors))
    plot_force_fig(force_sensors, ax, cc)
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plot_moment_fig(force_sensors, ax2, cc)
    plot_stance_index_fig(ax2, True, cc)
    ax2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=4, mode="expand", borderaxespad=0.)
    plt.show()

def plot_acc():
  ax, ax2, cc = prep_ax('Accelerometer')
  plot_gen3d_fig('acc_', ax, cc)
  ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
  plt.show()

def plot_gyro():
  ax, ax2, cc = prep_ax('Gyrometer')
  plot_gen3d_fig('rate_', ax, cc)
  ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
  plt.show()

def plot_vel():
  ax, ax2, cc = prep_ax('Linear velocity')
  plot_gen3d_fig('vel_', ax, cc)
  ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
  plt.show()

def plot_p():
  ax, ax2, cc = prep_ax('FF position (sensor)')
  plot_gen3d_fig('p_', ax, cc)
  ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
  plt.show()

def plot_ff():
  ax, ax2, cc = prep_ax('FF position (control)')
  plot_gen3d_fig('ff_t', ax, cc)
  ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
  plot_gen4d_fig('ff_q', ax2, cc)
  ax2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=4, mode="expand", borderaxespad=0.)
  plt.show()

def plot_rpy():
  ax, ax2, cc = prep_ax('RPY')
  plot_rpy_fig(ax, cc)
  ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
  plt.show()

def welcome():
    print "Available functions:"
    print "- plot_torque(joint_names)"
    print "- plot_encoder(joint_names)"
    print "- plot_velocity_actual(joint_names)"
    print "- plot_command(joint_names)"
    print "- plot_velocity_command(joint_names)"
    print "- plot_error(joint_names)"
    print "- plot_torque_error(joint_names)"
    print "- plot_command_encoder(joint_names)"
    print "- plot_velocity_command_actual(joint_names)"
    print "- plot_force(Left|Right/Foot|Hand)"
    print "- plot_acc|gyro|vel|p|ff()"

welcome()
