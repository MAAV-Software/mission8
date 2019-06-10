#!/usr/bin/python3

import sys
import os

# Add any zcm types that you want plotted in here
zcm_types_channels = [
    ('state_t', 'STATE'),
    ('imu_t', 'IMU'),
    ('imu_t', 'SIM_IMU'),
    ('lidar_t', 'HEIGHT_LIDAR'),
    ('plane_fit_t', 'PLANE_FIT'),
    ('global_update_t', 'GLOBAL_UPDATE'),
    ('groundtruth_inertial_t', 'GT_INERTIAL'),
    ('groundtruth_imu_t', 'GT_IMU'),
    ('groundtruth_world_t', 'GT_WORLD'),
    ('groundtruth_slamdrift_t', 'GT_SLAMDRIFT'),
    ('pid_error_t', 'PID_ERROR')
]

# CHANGE THIS TO CHANGE THE FREQUENCY OF UPDATES
loop_period = '33ms'

# Exctract the location of the zcm files globally
pathname = os.path.dirname(sys.argv[0])
cur_dir = os.path.abspath(pathname)
zcm_msgs_dir = cur_dir + '/../../msgtypes/zcm/'

valid_types = [
    'double',
    'quaternion_t',
    'matrix_t'
]

type_dimensions = {
    'double': 1,
    'quaternion_t': 4,
    'vector1_t': 1,
    'vector2_t': 2,
    'vector3_t': 3,
    'vector4_t': 4,
    'matrix_t': 1,
}

default_legends = {
    'vector1_t': ['x'],
    'vector2_t': ['x', 'y'],
    'vector3_t': ['x', 'y', 'z'],
    'vector4_t': ['w', 'x', 'y', 'z'],
    'quaternion_t': ['roll', 'pitch', 'yaw'],
    'matrix_t': ['matrix']
}

# Increase if we need more
max_vec_dim = 4
for i in range(max_vec_dim):
    valid_types.append('vector' + str(i) + '_t')


def main(zcm_types_channels, zcm_msgs_dir):
    print('Generating files ', zcm_types_channels, '\nfrom ', zcm_msgs_dir)
    parsed_zcm_types = {}
    for (type, channel) in zcm_types_channels:
        parsed_zcm_types[channel] = parse_zcm(type, zcm_msgs_dir)

    # print(parsed_zcm_types)
    generate_DataDict(parsed_zcm_types)
    generate_ZcmLoop(parsed_zcm_types)
    generate_ListWindow(parsed_zcm_types)


def parse_zcm(type, dir):
    zcm_file_dir = dir + type + '.zcm'
    zcm_file = open(zcm_file_dir, 'r')
    zcm_file_contents = zcm_file.read()
    zcm_msg = {'struct_name': '', 'vars': {}}

    lines = zcm_file_contents.splitlines()
    legend = []
    cov_indices = []
    for line in lines:
        striped_line = line.strip()
        if 'LEGEND:' in striped_line:
            starting_index = striped_line.find('[') + 1
            ending_index = striped_line.find(']')
            legend_str = striped_line[starting_index:ending_index]
            legend = legend_str.replace(' ', '').split(',')
        if 'COVARIANCE:' in striped_line:
            starting_index = striped_line.find('[') + 1
            ending_index = striped_line.find(']')
            cov_indices_str = striped_line[starting_index:ending_index]
            cov_indices = cov_indices_str.replace(' ', '').split(',')
            cov_indices = list(map(int, cov_indices))
        if not striped_line.startswith(('//', '/**', '*', '*/')) and striped_line:
            line_split = striped_line.split()
            if line_split[0] == 'struct':
                zcm_msg['struct_name'] = line_split[1]
            if line_split[0] in valid_types:
                local_type = line_split[0]
                var_name = line_split[1].replace(';', '')
                if not legend:
                    legend = default_legends[local_type]
                    legend = [var_name + '_' + elem for elem in legend]
                zcm_msg['vars'][var_name] = {
                    'type': local_type, 'legend': legend, 'cov_indices': cov_indices}
            # Clear legend
            legend = []
            cov_indices = []

    return zcm_msg


def generate_DataDict(dict):
    datadict_cpp = open(cur_dir + '/DataDict_generated.cpp', 'w')

    includes = ["\"DataDict.hpp\"",
                "\"AbstractData.hpp\"",
                "\"GaussianData.hpp\"",
                "\"QuaternionData.hpp\""]
    usings = []
    gen_header(includes, usings, datadict_cpp)

    print('void DataDict::initializeDictionary_generated()', file=datadict_cpp)
    print('{', file=datadict_cpp)
    for CHANNEL in dict:
        channel_type = dict[CHANNEL]
        vars = channel_type['vars']
        for var in vars:
            type = vars[var]['type']
            legend = vars[var]['legend']
            print('\tdict[\"' + CHANNEL + '_' + var + '\"]',
                  end='', file=datadict_cpp)
            if not vars[var]['cov_indices']:
                # Not Gaussian
                if type == 'quaternion_t':
                    print(' = std::shared_ptr<QuaternionData>(new QuaternionData(',
                          end='', file=datadict_cpp)
                else:
                    print(' = std::shared_ptr<AbstractData>(new AbstractData(',
                          end='', file=datadict_cpp)
            else:
                # Gaussian
                if type == 'quaternion_t':
                    print(' = std::shared_ptr<QuaternionData>(new QuaternionData(',
                          end='', file=datadict_cpp)
                else:
                    print(' = std::shared_ptr<GaussianData>(new GaussianData(',
                          end='', file=datadict_cpp)
            if type == 'quaternion_t':
                print(
                    '{', end='', file=datadict_cpp)
            else:
                print(str(type_dimensions[type]) +
                      ', {', end='', file=datadict_cpp)
            for i in range(len(legend) - 1):
                print('\"' + legend[i] + '\", ', end='', file=datadict_cpp)
            print('\"' + legend[len(legend) - 1] +
                  '\"', end='', file=datadict_cpp)
            print("}));", file=datadict_cpp)
    print('}', file=datadict_cpp)


def generate_ZcmLoop(dict):
    zcmloop_cpp = open(cur_dir + '/ZcmLoop_generated.cpp', 'w')

    includes = [
        "\"ZcmLoop.hpp\"",
        "\"AbstractData.hpp\"",
        "\"DataDict.hpp\"",
        "\"ZcmConversion.hpp\"",
        "\"GaussianData.hpp\"",
        "<chrono>",
        "<thread>",
        "<memory>",
        "<Eigen/Dense>",
        "<common/messages/MsgChannels.hpp>",
        "<common/utils/ZCMHandler.hpp>"
    ]

    for (zcm_type, channel) in zcm_types_channels:
        includes.append("<common/messages/" + zcm_type + ".hpp>")

    usings = ["namespace std::chrono"]
    gen_header(includes, usings, zcmloop_cpp)

    print('void ZcmLoop::run()\n{', file=zcmloop_cpp)

    # Start zcm
    for channel in dict:
        handler_name = channel + '_handler'
        type_name = dict[channel]['struct_name']
        print('\tZCMHandler<' + type_name + '> ' +
              handler_name + ';', file=zcmloop_cpp)
        print('\tzcm.subscribe(maav::' + channel + '_CHANNEL, &ZCMHandler<' +
              type_name + '>::recv, &' + handler_name + ');', file=zcmloop_cpp)
    print('\n\tzcm.start();\n', file=zcmloop_cpp)

    # While loop
    print('\twhile (RUNNING)\n\t{', file=zcmloop_cpp)

    conv_func = {
        'vector1_t': 'convertVector',
        'vector2_t': 'convertVector',
        'vector3_t': 'convertVector',
        'quaternion_t': 'convertQuaternion',
        'matrix_t': 'convertMatrix'
    }

    # Doo stuff
    for channel in dict:
        handler_name = channel + '_handler'

        # Check if zcm handler is ready
        print('\t\twhile(' + handler_name + '.ready()) {', file=zcmloop_cpp)
        print('\t\t\tdouble time = elapsedTime(static_cast<double>(' +
              handler_name + '.msg().utime) / 1e6);', file=zcmloop_cpp)

        foundCovMat = False
        #  Find the covariance matrix if applicable
        for var in dict[channel]['vars']:
            if var == 'covariance':
                foundCovMat = True
                print(
                    '\t\t\tEigen::MatrixXd covariance = convertMatrix(' +
                    handler_name + '.msg().covariance);',
                    file=zcmloop_cpp)

        # Add each variable to the dictionary
        for var in dict[channel]['vars']:
            type_name = dict[channel]['vars'][var]['type']

            # Disable until we have a proper way to handle these
            if type_name == 'matrix_t':
                continue

            # Check if we need to convert to a gaussian type
            cov_indices = dict[channel]['vars'][var]['cov_indices']
            if cov_indices:
                # Check that we have found a covariance matrix
                if not foundCovMat:
                    print('Covariance indices when there is no covariance matrix')
                else:
                    # Dynamic cast the shared pointer
                    print('\t\t\tstd::shared_ptr<GaussianData> ' +
                          channel + '_' + var +
                          ' = std::dynamic_pointer_cast<GaussianData>(dict_->dict["' +
                          channel + '_' + var + '"]);',
                          file=zcmloop_cpp)
                    block_size = str(cov_indices[0])
                    block_start = str(cov_indices[1])
                    print('\t\t\t' + channel + '_' + var + '->addData(std::move(' +
                          conv_func[type_name] + '(time, ' + handler_name + '.msg().' + var +
                          ')), covariance.block<' + block_size + ',' + block_size +
                          '>(' + block_start + ',' + block_start + '));',
                          file=zcmloop_cpp)
            else:
                print('\t\t\tdict_->dict["' + channel + '_' + var + '"]->addData(std::move(' +
                      conv_func[type_name] + '(time, ' + handler_name + '.msg().' + var + ')));', file=zcmloop_cpp)
        print('\t\t\t' + handler_name + '.pop();', file=zcmloop_cpp)
        print('\t\t}', file=zcmloop_cpp)
    print('\t\tstd::this_thread::sleep_for(' +
          loop_period + ');', file=zcmloop_cpp)
    print('\t}\n\n\tzcm.stop();\n}', file=zcmloop_cpp)


def generate_ListWindow(dict):
    list_window_cpp = open(cur_dir + '/ListWindow_generated.cpp', 'w')

    includes = [
        "\"ui_ListWindow.h\"",
        "\"ListWindow.h\""
    ]

    usings = []
    gen_header(includes, usings, list_window_cpp)

    print('void ListWindow::addChannels_generated()', file=list_window_cpp)
    print('{', file=list_window_cpp)
    for channel in dict:
        for var in dict[channel]['vars']:
            name = channel + '_' + var
            print('\tnew ChannelListItem(\"' + name + '\", \"' +
                  name + ':' + dict[channel]['vars'][var]['type'] + '\", list_);', file=list_window_cpp)
    print('}', file=list_window_cpp)


#     void ListWindow::addChannels_generated()
# {
#     new ChannelListItem("STATE, state_t, position", "STATE_position", ui_->listWidget);
#     new ChannelListItem("STATE, state_t, velocity", "STATE_velocity", ui_->listWidget);
#     new ChannelListItem("STATE, state_t, attitude", "STATE_attitude", ui_->listWidget);
# }


def gen_header(includes, usings, fout):
    print('// clang-format off', file=fout)
    print('/**\n * \tAUTOGENERATED FILE!! DO NOT EDIT!!!!\n */\n\n', file=fout)
    for include in includes:
        print("#include " + include, file=fout)
    print("", file=fout)
    for using in usings:
        print('using ' + using + ';\n', file=fout)


if __name__ == "__main__":
    main(zcm_types_channels, zcm_msgs_dir)
