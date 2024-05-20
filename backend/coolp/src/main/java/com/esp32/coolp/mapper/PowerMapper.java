package com.esp32.coolp.mapper;

import java.util.List;

import org.mapstruct.Mapper;

import com.esp32.coolp.dto.PowerDTO;
import com.esp32.coolp.entity.Readings;

@Mapper
public interface PowerMapper {
    PowerDTO map (Readings readings);

    List<PowerDTO> map(List<Readings> readings);
    
}
