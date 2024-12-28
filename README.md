**Order Server**

The order_server package provides two nodes, GoalPublisher and OrderStatusPublisher, to handle table order cancellation and robot navigation goals for a cafe or similar type environment.:

**Nodes Overview**

**1. goal_publisher**

The **'goal_publisher'** node acts as an action client that interacts with an action server,
**'SimpleNavigationServer'** in the 'simple_server' package.
It is responsible for sending navigation goals, monitoring feedback, and processing results.

Sends a NavigateToPose goal to the action server.
Publishes real-time feedback of the robot's current position during navigation.
Reports the final status of the navigation (e.g., succeeded, aborted, or canceled).

**Action Interface**

NavigateToPose (from nav2_msgs): Used to send and receive navigation-related data.

**Node Workflow**

Based on the orders specifies the target pose (goal).

Then sends the goal to the SimpleNavigationServer

Monitors feedback messages during navigation.

Reports the result of the goal execution.

Handle the failed executions by atampting the same locationg by 3 times

Check the order status and accordingly plan its upcoming targates.

**command**

    ros2 run order_server goal_publisher

**Example Logs**

    Place your order! Please press 'y' or 'n' along with your table number (e.g; y1,y2,n1,n2)!
    y1,y2,y3
    [INFO] [1735352637.950124043] [goal_publisher]: Sending goal to Kitchen...
    [INFO] [1735352637.952255562] [goal_publisher]: Goal accepted!
    [INFO] [1735352640.961753680] [goal_publisher]: Navigation succeeded!
    
    I am here to receive the order! Order is ready? Press 'y' or 'n'!
    y
    Order confirmed. Proceeding to tables...
    [INFO] [1735352646.561216775] [goal_publisher]: Sending goal to Table1...
    [INFO] [1735352646.565022473] [goal_publisher]: Goal accepted!
    [INFO] [1735352649.573505190] [goal_publisher]: Navigation succeeded!
    
    Want to receive the order! Press 'y' or 'n'!
    yNo response received. Skipping to next table...
    [INFO] [1735352659.585712994] [goal_publisher]: Sending goal to Table2...
    [INFO] [1735352659.589383904] [goal_publisher]: Goal accepted!
    [INFO] [1735352662.598376618] [goal_publisher]: Navigation failed!
    [INFO] [1735352662.599872316] [goal_publisher]: Navigation failed, will retry Table2...
    [INFO] [1735352662.600694107] [goal_publisher]: Retry attempt 2 for Table2...
    [INFO] [1735352664.604118076] [goal_publisher]: Sending goal to Table2...
    [INFO] [1735352664.607879095] [goal_publisher]: Goal accepted!
    [INFO] [1735352667.617484501] [goal_publisher]: Navigation failed!
    [INFO] [1735352667.619028460] [goal_publisher]: Navigation failed, will retry Table2...
    [INFO] [1735352667.619820184] [goal_publisher]: Retry attempt 3 for Table2...
    [INFO] [1735352669.621661615] [goal_publisher]: Sending goal to Table2...
    [INFO] [1735352669.625560472] [goal_publisher]: Goal accepted!
    [INFO] [1735352672.634607744] [goal_publisher]: Navigation succeeded!
    
    Want to receive the order! Press 'y' or 'n'!
    n
    No response received. Skipping to next table...
    [INFO] [1735352680.385089000] [goal_publisher]: Sending goal to Table3...
    [INFO] [1735352680.389212226] [goal_publisher]: Goal accepted!
    [INFO] [1735352683.398534604] [goal_publisher]: Navigation succeeded!
    
    Want to receive the order! Press 'y' or 'n'!
    y
    Order received. Processing next destination...
    [INFO] [1735352695.645493641] [goal_publisher]: Sending goal to Kitchen...
    [INFO] [1735352695.649412764] [goal_publisher]: Goal accepted!
    [INFO] [1735352698.658695308] [goal_publisher]: Navigation succeeded!
    [INFO] [1735352698.660511058] [goal_publisher]: Sending goal to Home...
    [INFO] [1735352698.663764721] [goal_publisher]: Goal accepted!
    [INFO] [1735352701.672816382] [goal_publisher]: Navigation succeeded!
    
    Place your order! Please press 'y' or 'n' along with your table number (e.g; y1,y2,n1,n2)!



**2. order_status**

The **OrderStatusPublisher** node is designed to manage and publish the cancellation status of table orders. It takes user input to cancel individual or all table orders and broadcasts the status over the **/order_status** topic.


**Publisher**: /order_status (std_msgs/String)
Publishes the cancellation status of table orders.

**Node Workflow**

Accepts user input in the format c1, c2, c3, or a comma-separated combination (e.g., c1,c2).

Publishes cancellation status messages for the specified tables or for all tables if all are selected.

Logs each published message for clarity.

**command**

    ros2 run order_server order_status

**Example Logs**

    Enter 'c' followed by the table number (e.g., c1, c2, c3). For all orders, enter all together (e.g., c1,c2,c3).
    c1
    [INFO] [1735353566.551615374] [order_status_publisher]: Published status: Y1 canceled

    Enter 'c' followed by the table number (e.g., c1, c2, c3). For all orders, enter all together (e.g., c1,c2,c3).
    c2
    [INFO] [1735353574.001851280] [order_status_publisher]: Published status: Y2 canceled
    
    Enter 'c' followed by the table number (e.g., c1, c2, c3). For all orders, enter all together (e.g., c1,c2,c3).
    c1,c2,c3
    [INFO] [1735353584.698653613] [order_status_publisher]: Published status: All orders canceled
    
    Enter 'c' followed by the table number (e.g., c1, c2, c3). For all orders, enter all together (e.g., c1,c2,c3).
