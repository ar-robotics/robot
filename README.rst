robot
=====

Kromium's code for communicating with the VR headset and the expansion board to control the robot. 

Important information
---------------------

* If you want to run any ROS2 node after making changes to any Python file inside the ``robot/src`` folder, you need to rebuild (``./colcon``).  

* If a new Python package is added to ``requirements.txt`` you would need to run ``./build_image`` again. 

Test physically (production)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Physically implies using the expansion board.

Make sure that ``production`` is set to ``true`` in the [config](/robot/src/controller/controller/config.json).

.. code-block:: bash

  # robot/
  ./run_docker_production


Enters the container as ``root`` to access the ``/dev/ttyUSB0`` (expansion board).
This device is also passed into the container.

Not testing physically (not production)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Not using expansion board.

Make sure that ``production`` is set to ``false`` in the [config](/robot/src/controller/controller/config.json).


.. code-block:: bash

  # robot/
  ./run_docker


If you get an error when trying to ``./run_all``, it might be because of non-root permissions.
You can try to delete the ``build``, ``install`` and ``log`` folders and run ``./colcon`` again.


Error: Access already in use when trying to start
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Make sure the Meta Quest 3 headset has closed the app, and run ``./kill_address``.


Prerequisites
-------------
Install Docker
^^^^^^^^^^^^^^

.. code-block:: bash

  # Add Docker's official GPG key:
  sudo apt-get update
  sudo apt-get install ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc

  # Add the repository to Apt sources:
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get update


Give permission to use Docker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  sudo groupadd docker
  sudo usermod -aG docker $USER


Installation
------------


Build the image
^^^^^^^^^^^^^^^^

.. code-block:: bash

  # robot/ (same directory as the Dockerfile)
  ./build_image



Running
-------

.. code-block:: bash

  # robot/
  ./run_docker # or ./run_docker_production starts the container
  ./colcon # builds the ROS2 packages
  ./run_all # starts all ROS2 nodes



Development
-----------

Start the container
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  # robot/
  ./run_docker

To exit the container use ``CTRL + D``.


When the container is running the ``robot/src`` folder will be shared between
the container and your local one. Changes made inside here will be apply to 
both. After changes has been made, make sure to run ``./colcon`` or 
``colcon build`` inside the ``robot/src`` folder. Now you can run using ``./run_all``
(if the package is included inside that file) or by running:

.. code-block:: bash

  # robot/src
  source install/local_setup.bash
  ros2 run package_name executable_variable


Testing
^^^^^^^

.. code-block:: bash

  # robot/src
  ./run_tests


Every unittest should output OK.

Create a new ROS2 package
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  # robot/src
  ros2 pkg create --build-type ament_python package_name


Documentation
-------------


HTML
^^^^
.. code-block:: bash

  # install sphinx
  sudo apt-get install python3-sphinx 
  pip3 install furo sphinxcontrib-jquery --break-system-packages
  cd docs/
  make clean
  make html


PDF
^^^

.. code-block:: bash
  
  sudo apt install latexmk texlive-latex-extra 
  cd docs/
  make latexpdf
