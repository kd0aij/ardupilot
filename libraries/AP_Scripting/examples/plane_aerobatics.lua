-- perform simple aerobatic manoeuvres in AUTO mode

local running = false

local roll_stage = 0

function do_axial_roll(m)
   -- constant roll rate axial roll
   if not running then
      running = true
      roll_stage = 0
      gcs:send_text(0, string.format("Starting roll"))
   end
   local roll_rate = m:param3()
   local throttle = m:param4()
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   if roll_stage == 0 then
      if roll_deg > 45 then
         roll_stage = 1
      end
   elseif roll_stage == 1 then
      if roll_deg > -5 and roll_deg < 5 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished roll r=%.1f p=%.1f", roll_deg, pitch_deg))
         vehicle:nav_scripting_update(true, throttle, 0, 0, 0)
         roll_stage = 2
         return
      end
   end
   if roll_stage < 2 then
      vehicle:nav_scripting_update(false, throttle, roll_rate, 0, 0)
   end
end

local loop_stage = 0

function do_loop(m)
   -- do one loop with controllable pitch rate and throttle
   if not running then
      running = true
      loop_stage = 0
      gcs:send_text(0, string.format("Starting loop"))
   end
   local pitch_rate = m:param3()
   local throttle = m:param4()
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   if loop_stage == 0 then
      if pitch_deg > 60 then
         loop_stage = 1
      end
   elseif loop_stage == 1 then
      if math.abs(roll_deg) < 90 and pitch_deg > -5 and pitch_deg < 5 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished loop p=%.1f", pitch_deg))
         vehicle:nav_scripting_update(true, throttle, 0, 0, 0)
         loop_stage = 2
         return
      end
   end
   if loop_stage < 2 then
      vehicle:nav_scripting_update(false, throttle, 0, pitch_rate, 0)
   end
end

function update()
   if vehicle:nav_scripting_active() then
      local cnum = mission:get_current_nav_index()
      local m = mission:get_item(cnum)
      local cmd = m:param1()
      if cmd == 1 then
         do_axial_roll(m)
      elseif cmd == 2 then
         do_loop(m)
      end
   else
      running = false
   end
   return update, 10
end

return update()
