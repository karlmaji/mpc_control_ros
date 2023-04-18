#!/usr/bin/env python
# -*- coding: utf-8 -*-



"""
    发布方实现：发布人的消息
        1.导包
        2.初始化ros节点
        3.创建发布者对象
        4.组织发布逻辑并发布数据
"""
'''
def publish_path():
    # 初始化ros节点
    rospy.init_node("path_publisher",anonymous=True)
    # 创建发布者对象
    pub= rospy.Publisher("global_path",Path,queue_size=10)
    # 设置循环速率
    rate = rospy.Rate(0.5)   # hz

    x = [0,1,2,3,4,5,6]
    y = [0,1,2,3,4,5,6]
    k = [0,1,2,3,4,5,6]
    heading = [0,1,2,3,4,5,6]
    

 # 循环发布数据
    while not rospy.is_shutdown():
        path = Path()   # 创建path类型的对象
        pose = PoseStamped()  # 创建PoseStamped类型的对象
        
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
   
        for i in range(len(x)):
            # path.x = x[i]
            # path.y = y[i]
            pose.pose.position.x = x[i]
            pose.pose.position.y = y[i]
            # path.k = k[i]
            # path.heading = heading[i]
            rospy.loginfo(path)
            # pose.pose.position.x = x[i]
            # pose.pose.position.y = y[i]
            # pose.pose.position.z = 0

            # pose.pose.orientation = heading[i]

            path.poses.append(pose)
        pub.publish(path)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_path()
        
    except rospy.ROSInterruptException:
        pass
'''
'''
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# Initialize the ROS node
rospy.init_node('global_path_publisher', anonymous=True)

# Create a publisher for the custom topic
path_publisher = rospy.Publisher('global_path', Path, queue_size=10)

# Function to generate the global path
def generate_global_path(coordinates, headings, curvatures, curvature_reciprocal, s):
    # Create a Path message
    global_path = Path()

    # Fill in the global path data
    global_path.header = Header()
    global_path.header.frame_id = "map"
    global_path.header.stamp = rospy.Time.now()

    # Add the coordinates, headings, and curvatures to the global path
    for (x, y), h, curvature, curvature_reciprocal, s in zip(coordinates, headings, curvatures, curvature_reciprocal, s):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = h
        pose.pose.orientation.w = curvature
        pose.pose.orientation.x = curvature_reciprocal
        pose.pose.orientation.y = s
        global_path.poses.append(pose)

    # Return the generated global path
    return global_path

def generate_global_path(coordinates, headings, curvatures, curvature_reciprocal, s):
    # Create a Path message
    global_path = Path()

    # Add the coordinates, headings, and curvatures to the global path
    for (x, y), h, curvature, curvature_reciprocal, s in zip(coordinates, headings, curvatures, curvature_reciprocal, s):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.header.seq = 0
        pose.pose.orientation.z = h
        pose.pose.orientation.w = curvature
        pose.pose.orientation.x = curvature_reciprocal
        pose.pose.orientation.y = s
        global_path.poses.append(pose)

    # Return the generated global path
    return global_path

# Main loop
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # Define the input data (coordinates, headings, and curvatures)
    coordinates = [(1, 2), (3, 4), (5, 6)]
    headings = [0.5, 0.7, 0.9]
    curvatures = [0.1, 0.2, 0.3]
    curvature_reciprocal = [1/0.1, 1/0.2, 1/0.3]
    s = [1, 2, 3]

    # Generate the global path
    global_path = generate_global_path(coordinates, headings, curvatures, curvature_reciprocal, s)

    # Publish the global path on the custom topic
    path_publisher.publish(global_path)

    # Sleep for the desired rate
    rate.sleep()
'''
"""
if __name__ == "__main__":

    # 2.初始化ros节点
    rospy.init_node("path_publisher",anonymous=True)
    # 3.创建发布者对象
    pub= rospy.Publisher("global_path",Path,queue_size=10)
    # 4.组织发布逻辑并发布数据
    # 4-1 创建Person数据
    p = Person()
    p.name = "奥特曼"
    p.age = 8
    p.height = 1.85
    # 4-2 创建 Rate 对象
    rate = rospy.Rate(1)
    # 4-3 循环发布数据
    while not rospy.is_shutdown():
        pub.publish(p)
        rospy.loginfo("发布的消息：%s,%d,%.2f",p.name,p.age,p.height)
        rate.sleep()
    def publish_path():
    # 创建发布者对象
    
    # 设置循环速率
    rate = rospy.Rate(10)   # 10hz

    x = [0,1,2,3,4,5,6]
    y = [0,1,2,3,4,5,6]
 # 循环发布数据
    while not rospy.is_shutdown():
        path = Path()   # 创建path类型的对象
        # pose = PoseStamped()  # 创建PoseStamped类型的对象
        
        # path.header.stamp = rospy.Time.now()
        # path.header.frame_id = "map"
   
        for i in range(len(x)):
            path.x = x[i]
            path.y = y[i]
            # pose.pose.position.x = x[i]
            # pose.pose.position.y = y[i]
            # pose.pose.position.z = 0

            # pose.pose.orientation = heading[i]

            # path.poses.append(pose)
        pub.publish(path)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_path()
        
    except rospy.ROSInterruptException:
        pass
"""
'''
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def publish_path():
    # Initialize the node
    rospy.init_node('path_publisher', anonymous=True)

    # Create a publisher for the path topic
    path_pub = rospy.Publisher('global_path', Float64MultiArray, queue_size=10)

    # Set the loop rate
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Define the path information
        x = [0, 1, 2, 3, 4, 5]  # x coordinates of the path
        y = [0, 0.5, 1, 1.5, 2, 2.5]  # y coordinates of the path
        heading = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # heading vector of the path
        curvature = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]  # curvature of the path
        curvature_inv = [100, 50, 33.33, 25, 20, 16.67]  # inverse curvature of the path
        s = [0, 0.5, 1, 1.5, 2, 2.5]  # distance along the path

        # Pack the path information into a Float64MultiArray message
        path_msg = Float64MultiArray()
        # path_msg.data = x + y + heading + curvature + curvature_inv + s
        path_msg.data.append(x[0])
        path_msg.data.append(y[0])
        path_msg.data.append(heading[0])
        path_msg.data.append(curvature[0])
        path_msg.data.append(curvature_inv[0])
        path_msg.data.append(s[0])

        # Publish the path message
        path_pub.publish(path_msg)

        # Sleep for the remaining time to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import math


def get_coordinate(index, map_width, map_resolution, origin_x, origin_y):
    """
    Returns the coordinates of the index-th cell
    """
    x = (index % map_width) * map_resolution + origin_x
    y = math.floor(index / map_width) * map_resolution + origin_y
    return x, y


def generate_global_path(coordinates, headings, curvatures, curvature_reciprocal, s):
    """
    Generate a global path from the given coordinates and headings using the
    piecewise cubic hermite interpolation method
    """
    # Create the Path object
    global_path = Path()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    global_path.header = header


    # Generate the PoseStamped objects and add them to the Path
    for i in range(len(coordinates)):
        coordinate = coordinates[i]
        heading = headings[i]
        curvature = curvatures[i]
        if curvature_reciprocal:
            curvature = 1 / curvature
        s_start = s[i]
        if i < len(s) - 1:
            s_end = s[i + 1]
        else:
            s_end = s_start + 2
        t = [s_start, s_end]
        pose = PoseStamped()
        pose.header = header
        pose.pose.position = Point(coordinate[0], coordinate[1], 0)
        pose.pose.orientation = Quaternion(0, 0, math.sin(heading / 2), math.cos(heading / 2))
        global_path.poses.append(pose)

    return global_path


def get_map_data(map_msg):
    """
    Extract map data from the OccupancyGrid message and return as a tuple
    """
    map_width = map_msg.info.width
    map_height = map_msg.info.height
    map_resolution = map_msg.info.resolution
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    grid_data = map_msg.data
    return map_width, map_height, map_resolution, origin_x, origin_y, grid_data


if __name__ == '__main__':
    rospy.init_node('demo01_pub_path_p')

    # Subscribe to the map topic to get the occupancy grid
    map_sub = rospy.Subscriber('/map', OccupancyGrid, get_map_data)

    # Wait until we have the occupancy grid
    while not rospy.has_param('/map'):
        rospy.sleep(0.1)

    # Get the map data
    map_width, map_height, map_resolution, origin_x, origin_y, grid_data = get_map_data(rospy.get_param('/map'))

    # Extract the coordinates of the non-obstacle cells from the occupancy grid
    coordinates = []
    for i in range(len(grid_data)):
        if grid_data[i] == 0:
            coordinates.append(get_coordinate(i, map_width, map_resolution, origin_x, origin_y))

    # Compute the headings and curvatures of the non-obstacle cells
    headings = []
    curvatures = []
    curvature_reciprocal = False
    for i in range(len(coordinates)):
        coordinate = coordinates[i]
'''

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf
# from tf.transformations import quaternion_from_euler


# Initialize the ROS node
rospy.init_node('global_path_publisher', anonymous=True)

# Create a publisher for the custom topic
path_publisher = rospy.Publisher('/global_path', Path, queue_size=10)

# Function to generate the global path
# def generate_global_path(coordinates, headings, curvatures, curvature_reciprocal, s):
def generate_global_path(coordinates, headings):
    # Create a Path message
    global_path = Path()

    global_path.header = Header()
    global_path.header.frame_id = "map"
    global_path.header.stamp = rospy.Time.now()

    # Add the coordinates, headings, and curvatures to the global path
    # for (x, y), h, curvature, curvature_reciprocal, s in zip(coordinates, headings, curvatures, curvature_reciprocal, s):
    for (x, y), h in zip(coordinates, headings):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        # pose.header = global_path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.header.seq = 0
        pose.header.frame_id = "map"
    
        quaternion = tf.transformations.quaternion_from_euler(0, 0, h)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        global_path.poses.append(pose)

    # Return the generated global path
    # return global_path
    return global_path

# Main loop
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    try:
        # Define the input data (coordinates, headings)
        coordinates = [
            (-26.75, 1.75), (-26.749999988849076, -0.2487734009912432), (-26.749999787680032, -2.0429901486678252), (-26.74999875771028, -3.6500406785670867), (-26.749995557923587, -5.086183614456627), (-26.74998792809518, -6.366600940751559), (-26.749972490237738, -7.505451228976547), (-26.749944568302503, -8.515920959549911), (-26.749898025968776, -9.41027398016703), (-26.749825122355873, -10.199899142060385), (-26.74971638549072, -10.895356155413424), (-26.749560503365355, -11.506419705205527), (-26.749344232417435, -12.042121868765417), (-26.74905232326783, -12.510792876310141), (-26.74866746354883, -12.92010025574704), (-26.748170237656705, -13.277086403015861), (-26.74753910326209, -13.588204619248318), (-26.746750384412223, -13.859353656022366), (-26.745778281058406, -14.095910809988458), (-26.744594894842542, -14.302763608145037), (-26.74317027097626, -14.484340125040516), (-26.74147245604659, -14.64463797317908), (-26.739467571581503, -14.787252007907453), (-26.737119903209294, -14.915400788060014), (-26.73439200524534, -15.031951833639482), (-26.73124482053991, -15.139445721810409), (-26.72763781542079, -15.24011906248278),
            (-26.72352912956428, -15.335926394762975), (-26.71887574062834, -15.428561045549342), (-26.713633643481536, -15.519474991549707), (-26.707758043861435, -15.60989776599798), (-26.701203566296137, -15.700854451347222), (-26.69392447612267, -15.793182799216401), (-26.685874915435814, -15.887549518868065), (-26.677009152801233, -15.984465775494288), (-26.667281846566272, -16.084301939588038), (-26.656648321602468, -16.187301628677336), (-26.645064859313184, -16.293595082699408), (-26.632489000740122, -16.40321191429212), (-26.61887986260244, -16.516093275279967), (-26.60419846610209, -16.632103480631834), (-26.58840807832908, -16.75104113116787), (-26.57147456610035, -16.872649776292686), (-26.553366762065856, -16.996628158032152), (-26.534056842915742, -17.122640077651056), (-26.513520719521978, -17.25032392612892), (-26.49173843884844, -17.3793019197712), (-26.46869459746288, -17.50918908223312), (-26.44437876648464, -17.639602014233517), (-26.418785927801615, -17.77016749223577), (-26.418785927801615, -17.77016749223577), (-26.326194721997535, -18.07976482279873), (-26.225721807589572, -18.386559986999202),
            (-26.117444551067905, -18.69036528416309), (-26.001444482545306, -18.99099496724689), (-25.87780725066238, -19.288265328440136), 
            (-25.74662257587063, -19.581994783818534), (-25.607984202111982, -19.872003957020333), (-25.46198984691299, -20.15811576191773), (-25.30874114991342, -20.440155484256245), (-25.14834361984777, -20.71795086223432), (-24.980906580000482, -20.991332165996795), (-24.806543112156024, -21.260132276015494), (-24.62536999906698, -21.52418676033092), (-24.43750766546827, -21.783333950629224), (-24.24308011767579, -22.03741501712927), (-24.0422148818278, -22.286274042254643), (-23.83504294087015, -22.52975809306702), (-23.6216986704642, -22.767717292437187), (-23.402319774141688, -23.000004888932743), (-23.1770472182847, -23.226477325402634), (-22.946025167944697, -23.44699430624373), (-22.70940092523421, -23.66141886333983), (-22.467324873189497, -23.869617420673553), (-22.219950429836707, -24.071459857626678), (-21.967434020024253, -24.26681957100671), (-21.709935076866334, -24.455573535872563), (-21.447616091003738, -24.637602365282717), (-21.180642735184687, -24.812790369163466), (-20.90918410504922, -24.981025612601496), (-20.633413135991614, -25.142199974016073), (-20.353507282579123, -25.296209203877748), (-20.069649583827776, -25.44295298493275), (-19.782030288029226, -25.582334995292893), (-19.490849279069952, -25.714262976292446), (-19.196319637704406, -25.83864880773984), (-18.898672792849524, -25.95540859415771), 
            (-18.59816587815484, -26.06446276687561), (-18.29509211839911, -26.165736208501), (-17.98979534163397, -26.259158408447213), (-17.682690062303166, -26.344663660969122), (-17.37428902712235, -26.42219132070007), (-17.065240682684088, -26.491686135184196), (-16.75637973970942, -26.553098679578493), (-16.44879490734923, -26.606385925827777), (-16.143918992213404, -26.65151198751159), (-15.843647948715653, -26.68844909260545), (-15.550497186470773, -26.717178850038156), (-15.267805553578661, -26.737693892691453), (-15.0, -26.75),
            (-15.0, -26.75), (-14.687732152419484, -26.749999999999996), (-14.363581500905232, -26.75),(-14.028160035359416, -26.75),(-13.682079745684199, -26.75),(-13.32595262178174, -26.749999999999996),(-12.960390653554215, -26.749999999999993),(-12.586005830903792, -26.75),(-12.20341014373263, -26.750000000000004), (-11.813215581942899, -26.75),(-11.416034135436764, -26.75),
            (-11.012477794116396, -26.749999999999993),(-10.603158547883956, -26.749999999999996),(-10.188688386641621, -26.749999999999996),(-9.769679300291545, -26.75),(-9.346743278735904, -26.750000000000007),
            (-8.920492311876856, -26.750000000000004),(-8.491538389616574, -26.750000000000004),(-8.06049350185722, -26.75),(-7.627969638500965, -26.75),(-7.194578789449974, -26.749999999999996),
            (-6.760932944606412, -26.749999999999993),(-6.327644093872452, -26.750000000000004),(-5.895324227150252, -26.75),(-5.464585334341983, -26.75),(-5.036039405349812, -26.75),
            (-4.610298430075906, -26.75),(-4.187974398422427, -26.75),(-3.769679300291546, -26.75),(-3.3560251255854285, -26.75),(-2.9476238642062427, -26.750000000000004),
            (-2.5450875060561513, -26.75),(-2.149028041037324, -26.75),(-1.760057459051926, -26.75),(-1.3787877500021248, -26.75),(-1.005830903790089, -26.75),
	        (-0.6417989103179809, -26.75),(-0.2873037594879695, -26.75),(0.05704255879777942, -26.75),(0.3906280546370966, -26.75),(0.7128407381278196, -26.75),
            (1.0230686193677794, -26.75),(1.32069970845481, -26.75),(1.605122015486744, -26.75),(1.875723550561414, -26.75),
            (2.1318923237766563, -26.75),(2.3730163452303032, -26.75),(2.5984836250201866, -26.75),(2.807682173244141, -26.75),(3.0, -26.75)
        ]
        headings = [
                    -1.5707963212160132, -1.5707962146740815, -1.5707956858880097, -1.5707940987529563, -1.5707903679342583, -1.5707827711435518, -1.5707686941658499, -1.5707442865684842, -1.57070399993373, -1.5706399736886985, -1.570541227105122, -1.5703926119121094, -1.5701734823962468, -1.5698560564139696, -1.5694034840350628, -1.5687677296381317, -1.5678875330378692, -1.5666869694340981, -1.5650754793304715, -1.5629506281350856, -1.560205096374432, -1.5567391380401954, -1.5524785107158678, -1.5473954212282304, -1.541526886676436, -1.5349828448775962, -1.5279377088212902, -1.5206047339033413, -1.5132001106323196, -1.5059083487967442, -1.4988591486050162, -1.4921198964564935, 
                    -1.4857014753946642, -1.4795716362565747, -1.4736702041353609, -1.4679223584925942, -1.4622484204414532, -1.4565701106878977, -1.4508139788417396, -1.4449128829255207, -1.4388062941692399, -1.4324400091433818, -1.4257656674489352, -1.4187403339945626, -1.411326313775466, -1.4034913152926898, -1.3952090551406482, -1.3864603909435362, -1.3772350745454371, 0.0, -1.2801931627209238, -1.2543122924487882, -1.2284283022836804, -1.2025413271686474, -1.176651502141372, -1.150758962328493, -1.1248638429419253, -1.0989662792740413, -1.0730664066951106, -1.047164360647643, -1.02126027664385, -0.995354290264868, -0.9694465371662199, -0.943537153099807, -0.9176262739748453, 
                    -0.8917140359988512, -0.8658005759879757, -0.8398860319950836, -0.8139705445267901, -0.788054258786715, -0.7621373286557456, -0.7362199234897627, -0.7103022393506092, -0.6843845169838642, -0.6584670697545635, -0.6325503258507551, -0.6066348903184613, -0.5807216338244754, -0.5548118162755082, -0.5289072542638344, -0.5030105413045535, -0.4771253283105076, -0.45125666777983114, -0.42541141748271627, -0.3995986863849808, -0.373830285057489, -0.34812111228839593, -0.32248936576216664, -0.2969564030643872, -0.27154599322677103, -0.24628257687031627, -0.22118797190713294, -0.1962756738668042, -0.1715414011415836, -0.14694759279419523, -0.12239763897452993, -0.09769138528626127, 
                    -0.07244339678777126, -0.045919348686022377, 0.0, 1.1377135706821229e-14, -1.0960069530029311e-14, 0.0, 0.0, 9.975970490171282e-15, 9.718499153580729e-15, -1.897894072547806e-14, -9.285817373082707e-15, 9.10497999383959e-15, 0.0, 1.7607026900762493e-14, -8.679566649995776e-15, 0.0, -8.478846389102543e-15, -1.6800241633393772e-14, 8.334793243941921e-15, 0.0, 8.242096773883857e-15, 0.0, 8.197481987863799e-15, 8.192661640934546e-15, -2.4598235145786433e-14, 8.217789540269728e-15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -8.699076165442923e-15, 8.825820592026675e-15, 
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                   ]
        # curvatures = [0.1, 0.2, 0.3]
        # curvature_reciprocal = [1/0.1, 1/0.2, 1/0.3]
        # s = [1, 2, 3]

        # Generate the global path
        # global_path = generate_global_path(coordinates, headings, curvatures, curvature_reciprocal, s)     Publish the global path on the custom topic
        global_path = generate_global_path(coordinates, headings) 
        path_publisher.publish(global_path)

        # Sleep for the desired rate
        rate.sleep()
    except rospy.ROSInterruptException:
        pass