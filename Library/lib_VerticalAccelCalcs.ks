function g {
  // g=GM/(R)^2
  return ship:body:mu/(ship:body:position:mag)^2.
}

function V_accel_inertial {
  local tmp_vec is VCRS(VCRS(-ship:body:position,ship:velocity:orbit),-ship:body:position):normalized.
  local centri_acc to VDOT(tmp_vec,ship:velocity:orbit)^2/-ship:body:position:mag.
  return centri_acc - g().
}
