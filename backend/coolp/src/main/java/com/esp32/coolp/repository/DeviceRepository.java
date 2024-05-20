package com.esp32.coolp.repository;

import com.esp32.coolp.entity.Device;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.JpaSpecificationExecutor;
import org.springframework.stereotype.Repository;

@Repository
public interface DeviceRepository extends JpaRepository<Device,Long>, JpaSpecificationExecutor<Device> {
}
