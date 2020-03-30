#!/bin/sh

services="dhcpcd resolv ntpd network ntpdate \
          growfs localmount fsck hostname"
cust_rc_services () {
  cp -vr ${NANO_WORLDDIR}/etc/runlevels ${NANO_CFGDIR}/ > ${NANO_LOG}/nano_rc_services.log 2>&1
  cd ${NANO_CFGDIR}/runlevels/default > ${NANO_LOG}/nano_rc_services.log 2>&1
  for service in ${services}
  do
      ln -sv /etc/init.d/${service} ${service} >> ${NANO_LOG}/nano_rc_services.log 2>&1
  done
}
