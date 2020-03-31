#!/bin/sh

# OpenRC services to enable
services="dhcpcd resolv ntpd network ntpdate \
	  localmount fsck hostname sshd ownership growfs"
# Additional / override services
cust_services="ownership ntpdate sshd"

cust_enable_rc_services () {
  cp -vr ${NANO_WORLDDIR}/etc/runlevels ${NANO_CFGDIR}/ > ${NANO_LOG}/nano_rc_services.log 2>&1
  cd ${NANO_CFGDIR}/runlevels/default > ${NANO_LOG}/nano_rc_services.log 2>&1
  for service in ${services}
  do
      ln -sv /etc/init.d/${service} ${service} >> ${NANO_LOG}/nano_rc_services.log 2>&1
  done
}

cust_add_rc_services ()
{
  for cust_service in ${cust_services}
  do
      cp -v ${topdir}/embedded/openrc/${cust_service} ${NANO_WORLDDIR}/etc/init.d/ >> ${NANO_LOG}/nano_rc_services.log 2>&1
      chmod 755 ${NANO_WORLDDIR}/etc/init.d/${cust_service}
  done
}
