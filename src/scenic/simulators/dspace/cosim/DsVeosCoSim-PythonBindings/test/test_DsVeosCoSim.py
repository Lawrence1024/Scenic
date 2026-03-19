import json
import os
import sys
import unittest
from collections import deque
from copy import deepcopy
from pathlib import Path
from threading import Thread
import logging
import tempfile

import helper_functions as helper

sys.path.append(str(Path(__file__).parent.parent / "src"))

from Bindings import (
    DSVEOSCOSIM_ETH_MESSAGE_MAX_LENGTH,
    CanMessage,
    CanMessageFlags,
    Command,
    ConnectionState,
    EthMessage,
    EthMessageFlags,
)
from CoSimClient import ConnectConfig, CoSimClient, CoSimSession


logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(threadName)s: %(message)s",
)
log = logging.getLogger(__name__)


VEOSPATH = None
LIBNAME = None
if sys.platform.startswith("win"):
    VEOSPATH = helper.find_windows_veos_installation("2025-B") / "veos.exe"
    LIBNAME = "DsVeosCoSim.dll"
else:
    VEOSPATH = "/opt/dspace/veos2025a/bin/veos"
    LIBNAME = "libDsVeosCoSim.so"

def find_or_build_co_sim_lib() -> Path:
    root = Path(__file__).parent.parent
    log.debug(f"Searching for {LIBNAME} under: {root}")
    DsVeosCoSim_lib = next(root.rglob(LIBNAME), None)
    if DsVeosCoSim_lib:
        log.info("Found DsVeosCoSim library at: %s", DsVeosCoSim_lib)
        return DsVeosCoSim_lib

    msg = f"Can\'t find {LIBNAME} in {root}"
    log.error(msg)
    raise FileNotFoundError(msg)


class OneCoSimComponent(unittest.TestCase):

    client = None
    connection = None
    connection_ctx = None

    @classmethod
    def setUpClass(self):
        log.info("[OneCoSimComponent] setUpClass: initializing VEOS and loading model")
        cwd = Path(__file__).parent
        self._osa = cwd / "OneCoSimComponent.osa"
        json_file = cwd / "SignalSource.json"

        log.debug("Reading configuration from: %s", json_file)
        with open(json_file, "r") as cfg:
            self.vpu_cfg = json.load(cfg)

        self.connection_cfg = ConnectConfig(serverName=self.vpu_cfg["Name"])

        log.info("Initializing VEOS...")
        helper.initialize_veos(VEOSPATH)

        log.info("Importing model from: %s", json_file)
        helper.veos_model_import(VEOSPATH, self._osa, json_file, create_new=True)

        log.info("Loading OSA: %s", self._osa)
        helper.load_veos_osa(VEOSPATH, self._osa)
        helper.veos_config_any(VEOSPATH, acceleration_factor=0, stop_time=1)

    @classmethod
    def tearDownClass(self):
        log.info("[OneCoSimComponent] tearDownClass: unloading VEOS model")
        try:
            helper.unload_veos_osa(VEOSPATH)
            OneCoSimComponent.connection_ctx.__exit__(None, None, None)
        except Exception as ex:
            log.warning("Failed to unload OSA file: %s (%s)", self._osa, ex)

    def test_client_connection(self):
        log.info("[test_client_connection] Starting test")
        OneCoSimComponent.client = CoSimClient()      
        self.assertEqual(ConnectionState.DISCONNECTED, OneCoSimComponent.client.GetConnectionState())
        log.debug("Client initial state is DISCONNECTED")

        OneCoSimComponent.connection_ctx = OneCoSimComponent.client.Connect(self.connection_cfg)
        OneCoSimComponent.connection = OneCoSimComponent.connection_ctx.__enter__() 

        log.info("Connected. State: %s", OneCoSimComponent.connection.client.GetConnectionState())
        self.assertEqual(ConnectionState.CONNECTED, OneCoSimComponent.connection.client.GetConnectionState())
        log.info("[test_client_connection] Completed")


    def test_connection(self):
        log.info("[test_connection] Starting connection test for server: %s", self.connection_cfg.serverName)

        def prop_vs_fcn(grp, prop, fcn):
            if grp in self.vpu_cfg and prop in self.vpu_cfg[grp]:
                expected = len(self.vpu_cfg[grp][prop])
                actual = len(fcn())
                log.debug("Property check: %s.%s expected=%d actual=%d", grp, prop, expected, actual)
                self.assertEqual(expected, actual)
            else:
                actual = len(fcn())
                log.debug("Property check: %s.%s not in config; expected=0 actual=%d", grp, prop, actual)
                self.assertEqual(0, actual)

        step_size = OneCoSimComponent.connection.GetStepSize()
        log.info("Step size: %s ns", step_size)
        self.assertEqual(self.vpu_cfg["StepSize"] * 10**9, step_size)

        # Network controllers
        prop_vs_fcn("Network", "CAN", OneCoSimComponent.connection.GetCanControllers)
        prop_vs_fcn("Network", "ETH", OneCoSimComponent.connection.GetEthControllers)
        prop_vs_fcn("Network", "LIN", OneCoSimComponent.connection.GetLinControllers)

        # IO signals
        prop_vs_fcn("IOSignals", "Incoming", OneCoSimComponent.connection.GetIncomingSignals)
        prop_vs_fcn("IOSignals", "Outgoing", OneCoSimComponent.connection.GetOutgoingSignals)

        log.info("[test_connection] Completed")


    # def test_simulation_start(self):
    #     def handle_command(session: CoSimSession, simulation_time: int, command: Command):
    #         match command:
    #             case Command.START:
    #                 session.connection.client.Disconnect()
    #                 return

    #         self.fail(f"Got unexpected Command: {command} at simutlation time: {simulation_time}")

    #     with CoSimClient().Connect(self.connection_cfg) as connection:
    #         connection.StartSimulation()
    #         connection.AttachToSimulation(handle_command)

    def test_send_and_receive_loopback(self):
        log.info("[test_send_and_receive_loopback] Begin CAN/ETH loopback test")

        def handle_command(session: CoSimSession, simulation_time: int, command: Command):
            match command:
                case Command.START:
                    session.step_cnt = 0
                    session.can_msgs = deque([])
                    session.eth_msgs = deque([])
                    log.info("Loopback START: initialized counters and queues")

                case Command.STEP:
                    if session.step_cnt % 2 == 0:  # Even -> transmit
                        log.debug("STEP %d (EVEN): Transmitting messages", session.step_cnt)

                        for controller in session.connection.GetCanControllers():
                            msg = CanMessage().FromController(controller)
                            msg.flags = msg.flags | CanMessageFlags.LOOPBACK
                            session.TransmitCanMessage(msg)
                            session.can_msgs.append(msg)
                            log.debug("TX CAN: %s", msg)

                        for controller in session.connection.GetEthControllers():
                            msg = EthMessage()
                            msg.controller_id = controller.id
                            msg.flags = EthMessageFlags.LOOPBACK
                            session.TransmitEthMessage(msg)
                            session.eth_msgs.append(msg)
                            log.debug("TX ETH: ctrl_id=%s flags=%s", controller.id, msg.flags)

                    else:  # Odd -> receive and verify last step's transmissions
                        log.debug("STEP %d (ODD): Receiving messages", session.step_cnt)

                        for controller in session.connection.GetCanControllers():
                            received_msg = session.ReceiveCanMessage()
                            log.debug("RX CAN (ctrl_id=%s): %s", controller.id, received_msg)
                            self.assertIsNotNone(received_msg)

                            send_msg = session.can_msgs.popleft()
                            try:
                                self.assertEqual(received_msg._fields_[1:], send_msg._fields_[1:])
                            except AssertionError:
                                log.error("CAN mismatch: sent=%s received=%s", send_msg, received_msg)
                                raise

                        for controller in session.connection.GetEthControllers():
                            received_msg = session.ReceiveEthMessage()
                            log.debug("RX ETH (ctrl_id=%s): %s", controller.id, received_msg)
                            self.assertIsNotNone(received_msg)

                            send_msg = session.eth_msgs.popleft()
                            try:
                                self.assertEqual(received_msg._fields_[1:], send_msg._fields_[1:])
                            except AssertionError:
                                log.error("ETH mismatch: sent=%s received=%s", send_msg, received_msg)
                                raise

                    session.step_cnt = session.step_cnt + 1

                case Command.STOP:
                    log.info("Loopback STOP: Disconnecting client")
                    # session.connection.StopSimulation()
                    session.connection.client.Disconnect()

        log.info("Starting simulation for loopback test")
        OneCoSimComponent.connection.StartSimulation()
        OneCoSimComponent.connection.AttachToSimulation(handle_command)
        log.info("[test_send_and_receive_loopback] Completed")


# class TwoCoSimComponents(unittest.TestCase):
#     @classmethod
#     def setUpClass(self):
#         log.info("[TwoCoSimComponents] setUpClass: preparing dual component VEOS model")
#         cwd = Path(__file__).parent
#         source_cfg_file = cwd / "SignalSource.json"
#         with open(source_cfg_file, "r") as signal_source:
#             self.signal_source = json.load(signal_source)

#         # Create inverse interface
#         self.signal_destination = deepcopy(self.signal_source)
#         self.signal_destination["Name"] = "SignalDestination"
#         self.signal_destination_conn_cfg = ConnectConfig(serverName=self.signal_destination["Name"])

#         io_signals = self.signal_destination["IOSignals"]
#         io_signals["Incoming"] = io_signals.pop("Outgoing")

#         destination_cfg_file = Path(tempfile.gettempdir()) / "SignalDestination.json"
#         with open(destination_cfg_file, "w") as signal_destination:
#             json.dump(self.signal_destination, signal_destination, indent=2)

#         self._osa = cwd / "TwoCoSimComponents.osa"

#         log.info("Initializing VEOS...")
#         helper.initialize_veos(VEOSPATH)

#         log.info("Importing model (source)...")
#         helper.veos_model_import(VEOSPATH, self._osa, source_cfg_file, create_new=True)

#         log.info("Importing model (destination)...")
#         helper.veos_model_import(
#             VEOSPATH,
#             self._osa,
#             destination_cfg_file,
#         )
#         log.info("Autoconnecting signals...")
#         helper.veos_model_autoconnect_signals(VEOSPATH, self._osa)

#         log.info("Loading OSA: %s", self._osa)
#         helper.load_veos_osa(VEOSPATH, self._osa)

#     @classmethod
#     def tearDownClass(self):
#         log.info("[TwoCoSimComponents] tearDownClass: unloading VEOS model")
#         try:
#             helper.unload_veos_osa(VEOSPATH)
#             OneCoSimComponent.connection_ctx.__exit__(None, None, None)
#         except Exception as ex:
#             log.warning("Failed to unload OSA file: %s (%s)", self._osa, ex)

#     def test_double_connection(self):
#         log.info("[test_double_connection] Opening two connections (src/dest)...")
#         with (
#             CoSimClient().Connect(ConnectConfig(serverName=self.signal_source["Name"])) as src,
#             CoSimClient().Connect(ConnectConfig(serverName=self.signal_destination["Name"])) as dest,
#         ):
#             log.info("Source state: %s", src.client.GetConnectionState().name)
#             log.info("Destination state: %s", dest.client.GetConnectionState().name)
#             self.assertEqual(ConnectionState.CONNECTED, src.client.GetConnectionState())
#             self.assertEqual(ConnectionState.CONNECTED, dest.client.GetConnectionState())
#         log.info("[test_double_connection] Completed")

#     def test_send_receive_io_signals(self):
#         log.info("[test_send_receive_io_signals] Begin IO signal exchange test")
#         def signal_source():
#             def handle_command(session: CoSimSession, simulation_time: int, command: Command):
#                 match command:
#                     case Command.STEP:
#                         outs = session.connection.GetOutgoingSignals()
#                         log.debug("Outgoing signals count: %d", len(outs))
#                         for io_signal in outs.values():
#                             arr = [simulation_time for _ in range(io_signal.length)]
#                             session.WriteOutgoingSignal(io_signal.id, arr)
#                             log.debug("Wrote outgoing signal id=%s len=%s val[0]=%s",
#                                       io_signal.id, io_signal.length, arr[0] if arr else None)

#                     case Command.STOP:
#                         session.connection.client.Disconnect()

#             with CoSimClient().Connect(
#                 ConnectConfig(
#                     clientName=self.signal_source["Name"],
#                     serverName=self.signal_source["Name"],
#                 )
#             ) as connection:
#                 connection.AttachToSimulation(handle_command)

#         def signal_destination():
#             def source_sim_time(connection, simulation_time):
#                 # the signal receiver is always one simulation step behind
#                 source_sim_time = simulation_time - connection.GetStepSize()
#                 if source_sim_time < 0:
#                     source_sim_time = 0
#                 return source_sim_time

#             def handle_command(session: CoSimSession, simulation_time: int, command: Command):
#                 match command:
#                     case Command.STEP:
#                         ins = session.connection.GetIncomingSignals()
#                         log.debug("Incoming signals count: %d", len(ins))
#                         for io_signal in ins.values():
#                             received_value = session.ReadIncomingSignal(io_signal)
#                             reference = [
#                                 source_sim_time(session.connection, simulation_time) for _ in range(io_signal.length)
#                             ]
#                             log.debug("Read incoming signal id=%s len=%s expected[0]=%s got[0]=%s",
#                                       io_signal.id, io_signal.length,
#                                       reference[0] if reference else None,
#                                       received_value[0] if received_value else None)
#                             self.assertEqual(received_value, reference)

#                     case Command.STOP:
#                         session.connection.client.Disconnect()

#             with CoSimClient().Connect(
#                 ConnectConfig(
#                     clientName=self.signal_destination["Name"],
#                     serverName=self.signal_destination["Name"],
#                 )
#             ) as connection:
#                 connection.AttachToSimulation(handle_command)

#         src = Thread(target=signal_source, name="SRC-IO")
#         dst = Thread(target=signal_destination, name="DST-IO")

#         src.start()
#         dst.start()

#         helper.veos_config_any(VEOSPATH, acceleration_factor=0, stop_time=1)

#         log.info("Starting VEOS simulation...")
#         helper.start_veos_simulation(VEOSPATH)

#         src.join()
#         dst.join()
#         log.info("[test_send_receive_io_signals] Completed")

#     def test_send_receive_can_frames(self):
#         log.info("[test_send_receive_can_frames] Begin CAN exchange test")
#         self.msgs = deque([])

#         def signal_source():
#             def handle_command(session: CoSimSession, simulation_time: int, command: Command):
#                 match command:
#                     case Command.STEP:
#                         ctrls = session.connection.GetCanControllers()
#                         log.debug("CAN controllers: %d", len(ctrls))
#                         for controller in ctrls:
#                             msg = CanMessage().FromController(controller)
#                             session.TransmitCanMessage(msg)
#                             self.msgs.append(msg)
#                             log.debug("TX CAN (ctrl_id=%s): %s", controller.id, msg)

#                     case Command.STOP:
#                         session.connection.client.Disconnect()

#             with CoSimClient().Connect(
#                 ConnectConfig(
#                     clientName=self.signal_source["Name"],
#                     serverName=self.signal_source["Name"],
#                 )
#             ) as connection:
#                 connection.AttachToSimulation(handle_command)

#         def signal_destination():
#             def handle_command(session: CoSimSession, simulation_time: int, command: Command):
#                 match command:
#                     case Command.STEP:
#                         ctrls = session.connection.GetCanControllers()
#                         log.debug("Expecting %d CAN frames this step", len(ctrls))
#                         for _ in ctrls:
#                             received_msg = session.ReceiveCanMessage()
#                             log.debug("RX CAN: %s", received_msg)
#                             self.assertIsNotNone(received_msg)

#                             send_msg = self.msgs.popleft()
#                             try:
#                                 self.assertEqual(received_msg._fields_[1:], send_msg._fields_[1:])
#                             except AssertionError:
#                                 log.error("CAN mismatch: sent=%s received=%s", send_msg, received_msg)
#                                 raise

#                     case Command.STOP:
#                         session.connection.client.Disconnect()

#             with CoSimClient().Connect(
#                 ConnectConfig(
#                     clientName=self.signal_destination["Name"],
#                     serverName=self.signal_destination["Name"],
#                 )
#             ) as connection:
#                 connection.AttachToSimulation(handle_command)

#         src = Thread(target=signal_source, name="SRC-CAN")
#         dst = Thread(target=signal_destination, name="DST-CAN")

#         src.start()
#         dst.start()

#         helper.veos_config_any(VEOSPATH, acceleration_factor=0, stop_time=1)

#         log.info("Starting VEOS simulation...")
#         helper.start_veos_simulation(VEOSPATH)

#         src.join()
#         dst.join()
#         log.info("[test_send_receive_can_frames] Completed")

#     def test_send_receive_eth_frames(self):
#         log.info("[test_send_receive_eth_frames] Begin ETH exchange test")
#         self.msgs = deque([])

#         def signal_source():
#             def handle_command(session: CoSimSession, simulation_time: int, command: Command):
#                 match command:
#                     case Command.STEP:
#                         ctrls = session.connection.GetEthControllers()
#                         log.debug("ETH controllers: %d", len(ctrls))
#                         for controller in ctrls:
#                             msg = session.MakeEthMsg(controller, data=list(range(DSVEOSCOSIM_ETH_MESSAGE_MAX_LENGTH)))
#                             session.TransmitEthMessage(msg)
#                             self.msgs.append(msg)
#                             log.debug("TX ETH (ctrl_id=%s) len=%s", controller.id, len(msg.data) if hasattr(msg, "data") else "n/a")

#                     case Command.STOP:
#                         session.connection.client.Disconnect()

#             with CoSimClient().Connect(
#                 ConnectConfig(
#                     clientName=self.signal_source["Name"],
#                     serverName=self.signal_source["Name"],
#                 )
#             ) as connection:
#                 connection.AttachToSimulation(handle_command)

#         def signal_destination():
#             def handle_command(session: CoSimSession, simulation_time: int, command: Command):
#                 match command:
#                     case Command.STEP:
#                         # NOTE: Logic kept identical to original (CAN-based receive path).
#                         # If you want, I can align this to ETH controllers and ReceiveEthMessage.
#                         ctrls = session.connection.GetCanControllers()
#                         log.debug("Expecting %d frames via CAN receive path (as in original)", len(ctrls))
#                         for _ in ctrls:
#                             received_msg = session.ReceiveCanMessage()
#                             log.debug("RX (via CAN receive path): %s", received_msg)
#                             self.assertIsNotNone(received_msg)

#                             send_msg = self.msgs.popleft()
#                             try:
#                                 self.assertEqual(received_msg._fields_[1:], send_msg._fields_[1:])
#                             except AssertionError:
#                                 log.error("Frame mismatch (ETH src vs CAN dst): sent=%s received=%s", send_msg, received_msg)
#                                 raise

#                     case Command.STOP:
#                         session.connection.client.Disconnect()

#             with CoSimClient().Connect(
#                 ConnectConfig(
#                     clientName=self.signal_destination["Name"],
#                     serverName=self.signal_destination["Name"],
#                 )
#             ) as connection:
#                 connection.AttachToSimulation(handle_command)

#         src = Thread(target=signal_source, name="SRC-ETH")
#         dst = Thread(target=signal_destination, name="DST-ETH")

#         src.start()
#         dst.start()

#         helper.veos_config_any(VEOSPATH, acceleration_factor=0, stop_time=1)

#         log.info("Starting VEOS simulation...")
#         helper.start_veos_simulation(VEOSPATH)

#         src.join()
#         dst.join()
#         log.info("[test_send_receive_eth_frames] Completed")


if __name__ == "__main__":
    DsVeosCoSim_lib = find_or_build_co_sim_lib()
    log.info("Starting unittest main runner...")
    unittest.main()