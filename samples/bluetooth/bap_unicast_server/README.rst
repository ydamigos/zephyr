.. zephyr:code-sample:: bluetooth_bap_unicast_server
   :name: Basic Audio Profile (BAP) Unicast Audio Server
   :relevant-api: bluetooth bt_audio bt_bap bt_pacs

   Use BAP Unicast Server functionality.

Overview
********

Application demonstrating the BAP Unicast Server functionality.
Starts advertising and awaits connection from a BAP Unicast Client.

This sample can be found under
:zephyr_file:`samples/bluetooth/bap_unicast_server` in the Zephyr tree.

Check the :zephyr:code-sample-category:`bluetooth` samples for general information.

Requirements
************

* BlueZ running on the host, or
* A board with Bluetooth Low Energy 5.2 support

Building and Running
********************

When building targeting an nrf52 series board with the Zephyr Bluetooth Controller,
use ``-DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf`` to enable the required ISO
feature support.

Building for an nrf52840dk
--------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/bap_unicast_server/
   :board: nrf52840dk/nrf52840
   :goals: build
   :gen-args: -DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf

Building for an nrf5340dk
-------------------------

You can build both the application core image and an appropriate controller image for the network
core with:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/bap_unicast_server/
   :board: nrf5340dk/nrf5340/cpuapp
   :goals: build
   :west-args: --sysbuild

If you prefer to only build the application core image, you can do so by doing instead:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/bap_unicast_server/
   :board: nrf5340dk/nrf5340/cpuapp
   :goals: build

In that case you can pair this application core image with the
:zephyr:code-sample:`bluetooth_hci_ipc` sample
:zephyr_file:`samples/bluetooth/hci_ipc/nrf5340_cpunet_iso-bt_ll_sw_split.conf` configuration.

Building for a simulated nrf52_bsim
-----------------------------------

Similarly to how you would for real HW, you can do:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/bap_unicast_server/
   :board: nrf52_bsim
   :goals: build
   :gen-args: -DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf

Note this will produce a Linux executable in :file:`./build/zephyr/zephyr.exe`.
For more information, check :ref:`this board documentation <nrf52_bsim>`.

Building for a simulated nrf5340bsim
------------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/bap_unicast_server/
   :board: nrf5340bsim/nrf5340/cpuapp
   :goals: build
   :west-args: --sysbuild
