------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and earlier: 
colorCorrectionFunction=function(_aShapeHandle_) 
	local version=simGetIntegerParameter(sim_intparam_program_version) 
	local revision=simGetIntegerParameter(sim_intparam_program_revision) 
	if (version<30104)and(revision<3) then 
		return _aShapeHandle_ 
	end 
	return '@backCompatibility1:'.._aShapeHandle_ 
end 
------------------------------------------------------------------------------ 
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
if (sim_call_type==sim_childscriptcall_initialization) then 
	simSetScriptAttribute(sim_handle_self,sim_childscriptattribute_automaticcascadingcalls,false) 
end 
if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 
if (sim_call_type==sim_childscriptcall_sensing) then 
	simHandleChildScripts(sim_call_type) 
end 
if (sim_call_type==sim_childscriptcall_actuation) then 
	if not firstTimeHere93846738 then 
		firstTimeHere93846738=0 
	end 
	simSetScriptAttribute(sim_handle_self,sim_scriptattribute_executioncount,firstTimeHere93846738) 
	firstTimeHere93846738=firstTimeHere93846738+1 
 



------------------------------------------------------------------------------ 
 
 
-- Kilobot Model
		-- K-Team S.A. --initial version
	    -- 2013.06.24
		
		-- Andrei G. Florea --curent kilolib (kilobotics.com) adapted version
		-- 09 November 2015
		
		-- Add your own program in function loop() , after the comment "user program code goes below"
	
	if (simGetScriptExecutionCount()==0) then 
	
		-- Check if we have a controller in the scene:
		i=0
		while true do
			h=simGetObjects(i,sim_handle_all)
			if h==-1 then break end
			if simGetObjectCustomData(h,4568)=='kilobotcontroller' then
				foundCtrller=true
				break
			end
			i=i+1
		end
		if not foundCtrller then
			simDisplayDialog('Error',"The KiloBot could not find a controller.&&nMake sure to have exactly one 'Kilobot_Controller' model in the scene.",sim_dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
		end
	
	
		-- save some handles
		KilobotHandle=simGetObjectAssociatedWithScript(sim_handle_self) 
	
		LeftMotorHandle=simGetObjectHandle('Kilobot_Revolute_jointLeftLeg')
		RightMotorHandle=simGetObjectHandle('Kilobot_Revolute_jointRightLeg')
	
		MsgSensorsHandle=simGetObjectHandle('Kilobot_MsgSensor')
		fullname=simGetNameSuffix(name)
	
		sensorHandle=simGetObjectHandle("Kilobot_Proximity_sensor")
		BaseHandle=simGetObjectHandle("BatHold")
	
		visionHandle=simGetObjectHandle("Vision_sensor")
		--BatGraphHandle=simGetObjectHandle("BatGraph") -- should be uncommented only for one robot
	
		half_diam=0.0165 -- half_diameter of the robot base
		
		RATIO_MOTOR = 10/255 -- ratio for getting 100% = 1cm/s or 45deg/s
	
		-- 4 constants that are calibration values used with the motors
		kilo_turn_right     = 200 -- value for cw motor to turn the robot cw in place (note: ccw motor should be off)
		kilo_turn_left    = 200 -- value for ccw motor to turn the robot ccw in place (note: cw motor should be off) 
		kilo_straight_right  = 200 -- value for the cw motor to move the robot in the forward direction
		kilo_straight_left = 200 -- value for the ccw motor to move the robot in the forward direction 


		-- message synopsys (follows kilobotics.com/message.h):
		-- msg = {type, data} where
		-- 		msg_type = uint8_t (MSG_TYPE_* see below)
		-- 		data = uint8_t[9] (table of 9 uint8_t)


		--special message codes (message_type_t)
		MSG_TYPE_NORMAL = 0
		MSG_TYPE_PAUSE = 4
		MSG_TYPE_VOLTAGE = 5
		MSG_TYPE_RUN = 6
		MSG_TYPE_CHARGE = 7
		MSG_TYPE_RESET = 8

        NR_ROBOT_ID_BYTES = 4 -- number of elements of the robotID table

        -- enum message data indexes
        INDEX_MSG_OWNER_UID  = 1
        INDEX_MSG_ROBOT_ID_0 = 2
        INDEX_MSG_ROBOT_ID_1 = 3
        INDEX_MSG_ROBOT_ID_2 = 4
        INDEX_MSG_ROBOT_ID_3 = 5

		--blink auxiliaries
		DELAY_BLINK = 100 --ms
		blink_in_progress = 0
		timexp_blink = 0

		-- enum directions
		DIR_STOP = 0
		DIR_STRAIGHT = 1
		DIR_LEFT = 2
		DIR_RIGHT = 3

		-- enum signal type
		SIGNAL_TYPE_GET                 = 1
		SIGNAL_TYPE_SET                 = 2
		SIGNAL_TYPE_GET_KNOWN_ROBOT_IDS = 3

		-- enum signal indexes
		INDEX_SIGNAL_TYPE   = 1 -- the type of signal received
		INDEX_SIGNAL_UID    = 2 -- the target / emitter UID on receive / send
		INDEX_SIGNAL_RECEIVE_MOTION = 3 -- used with set_motion() on SIGNAL_TYPE_SET
		-- red, green, blue values for RGB LED
		INDEX_SIGNAL_RECEIVE_LED_R  = 4
		INDEX_SIGNAL_RECEIVE_LED_G  = 5
		INDEX_SIGNAL_RECEIVE_LED_B  = 6


		--direction global vars
		direction = DIR_STOP
		direction_prev = DIR_STOP

	    -- for battery management
		battery_init = 8000000  -- battery initial charged value
		battery =  battery_init	-- battery simulator (~5h with 2 motors and 3 colors at 1 level)
		factMoving = 100		-- factor for discharging during moving
		moving = 0				-- for battery discharging	1: one motor, 2: 2 motors, 0: not moving
	    factCPU = 10			-- factor for discharging during cpu
		cpu=1					-- cpu state: sleep = 0, active = 1
		factLighting = 40		-- factor for discharging during lighting
		lighting=0				-- for battery managing
		bat_charge_status=0		-- battery charge status 0= no, 1, yes	
		
		charge_rate=400 		-- charge rate
		charge_max= battery_init-- battery_init*99.9/100 -- end of charge detection
	
		reset_substate	= 0

		-------------------------------------------------------------------------------------------------------------------------------------------
		--global variables
		-------------------------------------------------------------------------------------------------------------------------------------------
        distanceFromRobot = {} -- latest recorded distance from a robot that sent me a msg, indexed by robot uid
        -- list of robot uids stored as bitmasks (robotIDs[0] = id_s from 0 to 7, robotIDs[1] = ids from 8 to 15, ...)
        -- should be limited to 8 elements (8 * 8 = 64 ids) because of the 9 int limited message space
        robotIDs = {0, 0, 0, 0}

        -- construct bitmask table (0, 2, 4, 8, 16, .. 128)
        bitmask = {}
        bitmask[0] = 1
        bitmask[1] = 2
        bitmask[2] = 4
        bitmask[3] = 8
        bitmask[4] = 16
        bitmask[5] = 32
        bitmask[6] = 64
        bitmask[7] = 128

        share_known_robot_ids = true -- toggled at the receive of a setState signal

        -------------------------------------------------------------------------------------------------------------------------------------------
		-- Functions similar to C API
		-------------------------------------------------------------------------------------------------------------------------------------------

        ---------------------------------------------------------------------
        -- bitwise auxiliary functions from LuaBit
        -- used because of the lack of builtin support for bitwise functions
        -- in Lua 5.1 (used by V-REP)
        -- http://luaforge.net/projects/bit/

        function check_int(n)
            -- checking not float
            if(n - math.floor(n) > 0) then
                error("trying to use bitwise operation on non-integer!")
            end
        end

        function to_bits(n)
            check_int(n)
            if(n < 0) then
                -- negative
                return to_bits(bit.bnot(math.abs(n)) + 1)
            end
            -- to bits table
            local tbl = {}
            local cnt = 1
            while (n > 0) do
                local last = n % 2
                if(last == 1) then
                    tbl[cnt] = 1
                else
                    tbl[cnt] = 0
                end
                n = (n-last)/2
                cnt = cnt + 1
            end

            return tbl
        end

        function tbl_to_number(tbl)
            local n = table.getn(tbl)

            local rslt = 0
            local power = 1
            for i = 1, n do
                rslt = rslt + tbl[i]*power
                power = power*2
            end

            return rslt
        end

        function expand(tbl_m, tbl_n)
            local big = {}
            local small = {}
            if(table.getn(tbl_m) > table.getn(tbl_n)) then
                big = tbl_m
                small = tbl_n
            else
                big = tbl_n
                small = tbl_m
            end
            -- expand small
            for i = table.getn(small) + 1, table.getn(big) do
                small[i] = 0
            end

        end

        function bit_or(m, n)
            local tbl_m = to_bits(m)
            local tbl_n = to_bits(n)
            expand(tbl_m, tbl_n)

            local tbl = {}
            local rslt = math.max(table.getn(tbl_m), table.getn(tbl_n))
            for i = 1, rslt do
                if(tbl_m[i]== 0 and tbl_n[i] == 0) then
                    tbl[i] = 0
                else
                    tbl[i] = 1
                end
            end

            return tbl_to_number(tbl)
        end

        function bit_and(m, n)
            local tbl_m = to_bits(m)
            local tbl_n = to_bits(n)
            expand(tbl_m, tbl_n)

            local tbl = {}
            local rslt = math.max(table.getn(tbl_m), table.getn(tbl_n))
            for i = 1, rslt do
                if(tbl_m[i]== 0 or tbl_n[i] == 0) then
                    tbl[i] = 0
                else
                    tbl[i] = 1
                end
            end

            return tbl_to_number(tbl)
        end

        function bit_not(n)

            local tbl = to_bits(n)
            local size = math.max(table.getn(tbl), 32)
            for i = 1, size do
                if(tbl[i] == 1) then
                    tbl[i] = 0
                else
                    tbl[i] = 1
                end
            end
            return tbl_to_number(tbl)
        end

        --not originally available in LuaBit
        -- check that mask bit is set in x
        -- @param mask - int number (bitmask)
        -- @param x - int number
        -- @return true / false
        function bit_test(mask, x)
            if (bit_and(mask, x) == 0) then
                return false
            end
            return true
        end

        --end auxiliary bitwise functions
        ---------------------------------------------------------------------
        -- convert and store the given id int number into a bitmask used for storage in robotIDs table
        -- @param id - integer number (uint8_t in C)
        function setKnownRobotID(id)
            index = 1

            index = 1
            subtract = 0

            index = 2
            subtract = 8

            div_8 = math.floor(id / 8)
            index = div_8 + 1
            subtract = div_8 * 8

            -- compensate for the division of ids in 1 byte blocks (every id should now be within [0 - 7]
            id = id - subtract
            -- set the bit with number = id as 1 in the robotIDs slot with number = index
            robotIDs[index] = bit_or(robotIDs[index], bitmask[id])
        end

        -- convert the robotIDs binary encoded table into a table of decimal id numbers
        -- @return decTable : decimal tabel of known ids
        function getTableofKnownRobotIDs()
            decTable = {}

            for index = 1, 4, 1 do
                for i = 0, 7, 1 do
                    -- check that the i-th bit is set in the bitmask with number index from robotIDs
                    if (bit_test(bitmask[i], robotIDs[index])) then
                        table.insert(decTable, (index - 1) * 8 + i)
                    end
                end
            end

            return decTable
        end

        -- convert the robotIDs binary encoded table into a string representation of the decimal table of IDs
        -- @return decTable : decimal tabel of known ids
        function getStringTableofKnownRobotIDs()
            decTable = "{"

            for index = 1, 4, 1 do
                for i = 0, 7, 1 do
                    -- check that the i-th bit is set in the bitmask with number index from robotIDs
                    if (bit_test(bitmask[i], robotIDs[index])) then
                        decTable = decTable .. ((index - 1) * 8 + i) .. ", "
                    end
                end
            end

            return decTable .. "}"
        end

        -- join (bitwise OR) two robotID tables
        -- @param a, b : robotID tables
        -- @return joined table
        function joinRobotIdTables(a, b)
            result = a
            for i = 1, 4, 1 do
                result[i] = bit_or(a[i], b[i])
            end

            return result
        end

        function isRobotIdKnown(id)
            div_8 = math.floor(id / 8)
            index = div_8 + 1
            subtract = div_8 * 8

            -- compensate for the division of ids in 1 byte blocks (every id should now be within [0 - 7]
            id = id - subtract

            return bit_test(bitmask[id], robotIDs[index])
        end

        --called for each received message (of type == MSG_TYPE_NORMAL)
		-- @param msg_data (uint8_t[9] in C) : data contained in the message
		-- @param distance (uint8_t in C) : measured distance from the sender
		function message_rx(msg_data, distance)
			--simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": Message[1] = " .. msg_data[1] .. " received with distance = " .. distance)
            distanceFromRobot[msg_data[INDEX_MSG_OWNER_UID]] = distance
            -- if the id share period is not over
            if (share_known_robot_ids) then
                -- construct received robot id table
                receivedRobotIDs = {msg_data[INDEX_MSG_ROBOT_ID_0], msg_data[INDEX_MSG_ROBOT_ID_1], msg_data[INDEX_MSG_ROBOT_ID_2], msg_data[INDEX_MSG_ROBOT_ID_3]}

                -- bitwise OR the received table with the local table
                robotIDs = joinRobotIdTables(robotIDs, receivedRobotIDs)
            end
		end

		--called to construct every sent message
		--should be restricted to a 9 unsigned int table (as is the case for real kilobots)
		--@return msg = {type, data} where
		-- 		msg_type = uint8_t (MSG_TYPE_* see synopsys)
		-- 		data = uint8_t[9] (table of 9 uint8_t)
		function message_tx()
			--simAddStatusbarMessage("Message built");

            --publish known ids only if share_known is true
            if (share_known_robot_ids) then
                return {msg_type=MSG_TYPE_NORMAL, data={kilo_uid, robotIDs[1], robotIDs[2], robotIDs[3], robotIDs[4], 0, 0, 0, 0}}
            end

            return {msg_type=MSG_TYPE_NORMAL, data={kilo_uid, 0, 0, 0, 0, 0, 0, 0, 0}}
		end

		--called after each successfull message transmission
		function message_tx_success()
			--simAddStatusbarMessage("Message sent");
		end

		--function called only once at simmulation start (or after clicking Reset from the controller)
		--You should put your inital variable you would like to reset inside this function for your program.
		function setup()
			-- get unique robot id from "robotID" parameter
			--kilo_uid = simGetScriptSimulationParameter(sim_handle_self, "robotID", false)

			--kilo_uid is determined from name suffix in order to automatically add any number of robots with unique UIDs
			kilo_uid = simGetNameSuffix(simGetScriptName(sim_handle_self))
			--the source robot has nameSuffix == -1 so his uid should be 0 and clones start from 1
			if (kilo_uid == -1) then
				kilo_uid = 0
			else
				kilo_uid = kilo_uid + 1
			end
			simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": kilo_uid=" .. kilo_uid)

            -- set distance to me = 0
            distanceFromRobot[kilo_uid] = 0
            -- set myself as known robot id for other robots
            setKnownRobotID(kilo_uid)

            simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": knownRobots = " .. getStringTableofKnownRobotIDs())
		end

		function loop()
		--/////////////////////////////////////////////////////////////////////////////////////
		--//user program code goes below.  this code needs to exit in a resonable amount of time
		--//so the special message controller can also run
		--/////////////////////////////////////////////////////////////////////////////////////

			--read signal value if available
			local command = simGetStringSignal('signal')

			-- if not nil
			if (command) then
				params = simUnpackInts(command)
				-- if this signal was destined for me
				if (kilo_uid == params[INDEX_SIGNAL_UID]) then
					-- clear signal to be able to notice a new signal
					simClearStringSignal('signal')
					simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": Command received = " .. params[INDEX_SIGNAL_TYPE])

					-- if command == getState
					if (params[INDEX_SIGNAL_TYPE] == SIGNAL_TYPE_GET) then
						-- send a reply
						simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": Sending a packed msg with sensor data")
                        -- construct variables for keys and values separately
                        dist_keys = {}
                        dist_values = {}

                        for k, v in pairs(distanceFromRobot) do
                            table.insert(dist_keys, k)
                            table.insert(dist_values, v)
                        end

                        -- reinitialize distanceFromRobot
                        distanceFromRobot = {}
                        -- set distance to me = 0
                        distanceFromRobot[kilo_uid] = 0

						simSetStringSignal('reply_signal', simPackInts( {kilo_uid, get_ambient_light() * 100} ) .. "|" ..
                            simPackInts(dist_keys) .. "|" .. simPackInts(dist_values) )

                    -- if command == getKnownRobotIds
                    elseif (params[INDEX_SIGNAL_TYPE] == SIGNAL_TYPE_GET_KNOWN_ROBOT_IDS) then
                        -- send a reply
						simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": Sending a packed msg with known robot IDs")

                        -- stop the ID share procedure
                        share_known_robot_ids = false
                        simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": robot id share ended")
                        simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": knownRobots = " .. getStringTableofKnownRobotIDs())

						simSetStringSignal('reply_signal', kilo_uid .. "|" ..  simPackInts(getTableofKnownRobotIDs()))

					-- if command == setState
					elseif (params[INDEX_SIGNAL_TYPE] == SIGNAL_TYPE_SET) then
                        -- send a reply
						simAddStatusbarMessage(simGetScriptName(sim_handle_self) .. ": Changing my state")
						set_motion(params[INDEX_SIGNAL_RECEIVE_MOTION])
						set_color(params[INDEX_SIGNAL_RECEIVE_LED_R], params[INDEX_SIGNAL_RECEIVE_LED_G], params[INDEX_SIGNAL_RECEIVE_LED_B])
						simSetStringSignal('reply_signal', simPackInts({kilo_uid, 1})) -- 1 == i changed my state

					-- if command == unrecognized
					else
					-- send a reply
						simSetStringSignal('reply_signal', simPackInts({kilo_uid, 0})) -- 0 == i don't know that you want
					end
				end
			end
		--////////////////////////////////////////////////////////////////////////////////////
		--//END OF USER CODE
		--////////////////////////////////////////////////////////////////////////////////////
		end
		
		-------------------------------------------------------------------------------------------------------------------------------------------
		
	    -- Returns the value of ambient light
		function get_ambient_light()
			result,auxValues1,auxValues2=simReadVisionSensor(visionHandle)
			if (auxValues1) then

				return auxValues1[11] -- return average intensity
			else
				return -1
			end
		end

		-- Set direction of motion
		-- @param dir_new : new direction to go, one of (DIR_STOP, DIR_STRAIGHT, DIR_LEFT, DIR_RIGHT)
		function set_motion(dir_new)
			if (dir_new == DIR_STOP) then
				set_motor(0, 0)

			elseif (dir_new == DIR_STRAIGHT) then
				set_motor(kilo_straight_right, kilo_straight_left)

			elseif (dir_new == DIR_LEFT) then
				set_motor(0, kilo_turn_left)

			elseif (dir_new == DIR_RIGHT) then
				set_motor(kilo_turn_right, 0)
			end


			direction_prev = direction
			direction = dir_new
		end

		--blinks LED for DELAY_BLINK interval
		-- @param r : red value (0-3)
		-- @param g : green value (0-3)
		-- @param b : blue value (0-3)
		function blink(r, g, b)
			blink_in_progress = 1
			--simAddStatusbarMessage("<-- time = ".. simGetSimulationTime());
			timexp_blink = simGetSimulationTime() + DELAY_BLINK / 1000.0
			--simAddStatusbarMessage("-- timexp_blink = ".. timexp_blink);
			set_color(r, g, b)
		end

		-- Set motor speed PWM values for motors between 0 (off) and 255 (full on, ~ 1cm/s) for cw_motor and ccw_motor 
		function set_motor(cw_motor,ccw_motor)
		-- Set speed
			simSetJointTargetVelocity(RightMotorHandle,ccw_motor*RATIO_MOTOR)
			simSetJointTargetVelocity(LeftMotorHandle,cw_motor*RATIO_MOTOR)
	
			-- for battery managing
			if ((cw_motor == 0) and (ccw_motor==0)) then
				moving=0
			elseif ((cw_motor == 0) or (ccw_motor==0)) then
			  moving=1	
			else
			-- both moving
			  moving=2
			end
	
		end

		-- Set LED color
		-- @param r : red value (0-3)
		-- @param g : green value (0-3)
		-- @param b : blue value (0-3)
		function set_color(r,g,b)
			simSetShapeColor(colorCorrectionFunction(BaseHandle),"BODY",0,{r*0.6/3.0+0.1, g*0.6/3.0+0.1, b*0.6/3.0+0.1})
			lighting=r+g+b
		end
	
		-------------------------------------------------------------------------------------------------------------------------------------------
		-- END Functions similar to C API
		-------------------------------------------------------------------------------------------------------------------------------------------
	
		enable_tx = 0 -- to turn on/off the transmitter
		senderID = nil
	
		special_mode = 1
		run_program = 0
		special_mode_message = MSG_TYPE_RESET --start the robot with a reset (to call setup() only once during start-up)
	
		function receive_data()
			-- receive latest message and process it
			
			data,senderID,dataHeader,dataName=simReceiveData(0,"Message",MsgSensorsHandle)
			
			if (data ~= nil) then
				senderHandle= simGetObjectAssociatedWithScript(senderID)
				udata=simUnpackInts(data)
				
				--reconstruct message structure
				message = {msg_type = udata[1], 
							data = {udata[2], udata[3], udata[4], udata[5], udata[6], udata[7], udata[8], udata[9], udata[10]}}

				--simAddStatusbarMessage("message[msg_type] = " .. message["msg_type"]);

				-- special message
				if (message["msg_type"] > MSG_TYPE_NORMAL) then
					special_mode_message = message["msg_type"]
					special_mode = 1
	
				else
					--normal message processing

					result, distance, detectedPoint = simCheckProximitySensor(sensorHandle,senderHandle)
					
					-- if the distance was extracted corectly
					if (result == 1) then
						distance = (distance + half_diam) * 1000  -- distance in mm + 1/2diameter of robot
						-- send the message contents to user processing with distance
						message_rx(message["data"], distance)
					end
				end
			else	
				--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." no received data")
			end
		end
	
		irstart=simGetSimulationTime()
	
		function send_data() -- send data from ir every 0.2s, at a max distance of 7cm (ONLY if kilobot is in normal running state)
			newir=simGetSimulationTime()
			--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." enable_tx:"..enable_tx.."  irstart:"..irstart.."  newir:"..newir)
			if ((enable_tx==1) and (newir-irstart>0.2) and (run_program == 1)) then
				local new_msg = message_tx() --the user function is resposible for composing the message
				--serialize (msg_type, data[1], data[2], ... data[9]) and send message
				simSendData(sim_handle_all,0,"Message",
					simPackInts({new_msg["msg_type"], new_msg["data"][1], new_msg["data"][2], new_msg["data"][3], new_msg["data"][4], new_msg["data"][5],
						new_msg["data"][6], new_msg["data"][7], new_msg["data"][8], new_msg["data"][9]}),
					MsgSensorsHandle,0.07,3.1415,3.1415*2,0.8)

				--message transmission ok => notify the user
				message_tx_success()

				--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." sent a mesage")
				irstart=newir
	        end 
			
		end
	
		-- Measure battery voltage, returns voltage in .01 volt units
		-- for example if 394 is returned, then the voltage is 3.94 volts 
		function measure_voltage()
			return battery*420/battery_init
		end
	
		-- Measure if battery is charging, returns 0 if no, 1 if yes 
		function measure_charge_status()
			return bat_charge_status
		end
	
		substate=0 -- sub state for state machine of message
	
		-- battery management
		function update_battery()
			dt=simGetSimulationTimeStep()
			battery=battery-factLighting*lighting-factMoving*moving-factCPU*cpu
			--simSetGraphUserData(BatGraphHandle,"Battery",battery)  -- should be uncommented only for one robot
		end
	
		delay_start=simGetSimulationTime()
	
			-- wait for x milliseconds  global variable delay_start should be initialised with:  delay_start=simGetSimulationTime()
		function _delay_ms(x)
			--simWait(x/1000.0,true)
				if ((simGetSimulationTime()-delay_start)>=(x/1000.0)) then
					return 1
				end
			return 0
		end
	
		-------------------------------------------------
		-- other initialisations
	
		--kilo_uid = math.random(0, 255) -- set robot id
	
		-- get number of other robots
	
		NUMBER_OTHER_ROBOTS=0
		objIndex=0
		while (true) do
			h=simGetObjects(objIndex,sim_object_shape_type)
			if (h<0) then
				break
			end
			objIndex=objIndex+1
			--simAddStatusbarMessage("objIndex: "..objIndex)
			if ((simGetObjectCustomData(h,1834746)=="kilobot") and (KilobotHandle ~= h))then
				NUMBER_OTHER_ROBOTS=NUMBER_OTHER_ROBOTS+1
				--simAddStatusbarMessage("NUMBER_OTHER_ROBOTS: "..NUMBER_OTHER_ROBOTS)
			end
		end	
	
		--simAddStatusbarMessage("number of robots found: "..robotnb)
	
	
	end
	
	---------------------------------------------------------------------------
	---------------------------------------------------------------------------
	-- main script loop
	
	simHandleChildScripts(sim_call_type)
	
	update_battery() -- update battery value
	
	receive_data() -- received data by ir
	
	send_data() -- send data by ir
	
	--special message controller, handles controll messages like sleep and resume program
	if(special_mode==1) then
	
		run_program=0
	
		special_mode=0
		set_motor(0,0)
	
		-- modes for different values of special_mode_message	 
		--0x01 bootloader (not implemented)
		--0x02 sleep (not implemented)
		--0x03 wakeup, go to mode 0x04
		--0x04 Robot on, but does nothing active
		--0x05 display battery voltage
		--0x06 execute program code
		--0x07 battery charge
		--0x08 reset program
	
	
		if(special_mode_message==0x02) then
		  -- sleep	
			wakeup=0
			--enter_sleep();//will not return from enter_sleep() untill a special mode message 0x03 is received	
		elseif((special_mode_message==0x03)or(special_mode_message==MSG_TYPE_PAUSE)) then
		  --wakeup / Robot on, but does nothing active
			enable_tx=0
			
			--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." substate: "..substate) 
	
			-- make the led blink
			if (substate==0) then	
				set_color(3,3,0)
				substate=substate+1
				delay_start=simGetSimulationTime()
			elseif (substate==1) then
				if (_delay_ms(50)==1) then 
					substate=substate+1
				end
			elseif (substate==2) then
				set_color(0,0,0)
				substate=substate+1
				delay_start=simGetSimulationTime()
			elseif (substate==3) then
				if (_delay_ms(1300)==1) then
					substate=0
				end
			end
	
			enable_tx=1
			special_mode=1
		
		elseif(special_mode_message==MSG_TYPE_VOLTAGE) then
		 -- display battery voltage
			enable_tx=0
	
			if(measure_voltage()>400) then
				set_color(0,3,0)
			elseif(measure_voltage()>390) then
				set_color(0,0,3)
			elseif(measure_voltage()>350) then
				set_color(3,3,0)
			else
				set_color(3,0,0)
			end
	
			enable_tx=1
			--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." Voltage: "..measure_voltage().."  battery:"..battery)
		elseif (special_mode_message==MSG_TYPE_RUN) then
			--execute program code
			enable_tx=1
			run_program=1
			substate = 0
			--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." special mode Run") 
			--no code here, just allows special_mode to end 
	
		elseif (special_mode_message==MSG_TYPE_CHARGE) then
		 --battery charge
			enable_tx=0
			--if(measure_charge_status()==1) then
			
			if (battery<charge_max) then
				if (substate==0) then	
					set_color(1,0,0)
					substate=substate+1
					delay_start=simGetSimulationTime()
				elseif (substate==1) then
					if (_delay_ms(50)==1) then 
						substate=substate+1
					end
				elseif (substate==2) then
					set_color(0,0,0)
					substate=substate+1
					delay_start=simGetSimulationTime()
				elseif (substate==3) then
					if (_delay_ms(300)==1) then
						substate=0
					end
				end
			
				battery=battery+charge_rate
			
				if (battery>battery_init) then
					battery=battery_init
				end
			end
			special_mode=1
	
	
	
			enable_tx=1
		elseif (special_mode_message==MSG_TYPE_RESET) then
			
			if (reset_substate==0)	then
			--reset
			enable_tx=0
			setup()
			run_program = 0
			special_mode_message = MSG_TYPE_RESET
			reset_substate = reset_substate + 1
			-- wait some time for stopping messages
			--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." start resetting") 
			delay_reset=simGetSimulationTime()
			elseif (simGetSimulationTime()-delay_reset>=1.5) then  	
				special_mode_message = 3
				reset_substate	= 0
			else
				while (simReceiveData(0,"Message",MsgSensorsHandle)) do 
					--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." empty message buffer") 
				end
			end
			
			special_mode = 1
	
		end
	
	end
	
	if(run_program==1) then
		
		--simAddStatusbarMessage(simGetScriptName(sim_handle_self).." process Run") 
		if (blink_in_progress == 1) then
			if (simGetSimulationTime() >= timexp_blink) then
				simAddStatusbarMessage("--> time = ".. simGetSimulationTime());
				blink_in_progress = 0;
				set_color(0, 0, 0);
			end
		else
			loop()
		end
	
	end
	
	
	if (simGetSimulationState()==sim_simulation_advancing_lastbeforestop) then
		-- Put some restoration code here
		set_color(0,0,0)
	end
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 






