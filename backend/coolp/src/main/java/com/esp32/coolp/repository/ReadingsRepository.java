package com.esp32.coolp.repository;

import com.esp32.coolp.entity.Readings;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.JpaSpecificationExecutor;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

import java.sql.Timestamp;
import java.util.List;

@Repository
public interface ReadingsRepository extends JpaRepository<Readings,Long>, JpaSpecificationExecutor<Readings> {
    List<Readings> findAllByDeviceId (Long deviceId);

    List<Readings> findAllByDeviceIdAndCreatedAtBetween(Long deviceId, Timestamp startDate, Timestamp endDate);


    @Query(value = "SELECT * FROM Readings WHERE device_id = :deviceId ORDER BY created_at DESC LIMIT 1", nativeQuery = true)
    Readings findLatestByDeviceId(Long deviceId);

}
