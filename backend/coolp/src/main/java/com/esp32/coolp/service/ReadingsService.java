package com.esp32.coolp.service;

import com.esp32.coolp.entity.Readings;
import com.esp32.coolp.repository.ReadingsRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.sql.Timestamp;
import java.util.List;

@Service
public class ReadingsService {
    @Autowired
    ReadingsRepository readingsRepository;

    public List<Readings> findAllByDeviceId (Long deviceId){
        return readingsRepository.findAllByDeviceId(deviceId);
    }
    public List<Readings> findAllByDeviceIdAndDateRange(Long deviceId, Timestamp startDate, Timestamp endDate) {
        return readingsRepository.findAllByDeviceIdAndCreatedAtBetween(deviceId, startDate, endDate);
    }
    public Readings findCurrentData(Long deviceId) {
        return readingsRepository.findLatestByDeviceId(deviceId);
    }

    public Readings save(Readings readings) {
        return  readingsRepository.save(readings);
    }
}
