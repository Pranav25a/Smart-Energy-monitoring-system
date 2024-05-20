package com.esp32.coolp.mapper;

import com.esp32.coolp.dto.PowerDTO;
import com.esp32.coolp.entity.Readings;
import java.util.ArrayList;
import java.util.List;
import javax.annotation.processing.Generated;

@Generated(
    value = "org.mapstruct.ap.MappingProcessor",
    date = "2024-03-25T15:54:29+0530",
    comments = "version: 1.4.2.Final, compiler: Eclipse JDT (IDE) 3.37.0.v20240206-1609, environment: Java 17.0.10 (Eclipse Adoptium)"
)
public class PowerMapperImpl implements PowerMapper {

    @Override
    public PowerDTO map(Readings readings) {
        if ( readings == null ) {
            return null;
        }

        PowerDTO powerDTO = new PowerDTO();

        powerDTO.setCreatedAt( readings.getCreatedAt() );
        powerDTO.setPowerunit( readings.getPowerunit() );

        return powerDTO;
    }

    @Override
    public List<PowerDTO> map(List<Readings> readings) {
        if ( readings == null ) {
            return null;
        }

        List<PowerDTO> list = new ArrayList<PowerDTO>( readings.size() );
        for ( Readings readings1 : readings ) {
            list.add( map( readings1 ) );
        }

        return list;
    }
}
