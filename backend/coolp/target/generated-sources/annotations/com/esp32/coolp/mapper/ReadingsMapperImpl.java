package com.esp32.coolp.mapper;

import com.esp32.coolp.dto.ReadingsDTO;
import com.esp32.coolp.entity.Readings;
import java.util.ArrayList;
import java.util.List;
import javax.annotation.processing.Generated;
import org.springframework.stereotype.Component;

@Generated(
    value = "org.mapstruct.ap.MappingProcessor",
    date = "2024-03-25T15:54:29+0530",
    comments = "version: 1.4.2.Final, compiler: Eclipse JDT (IDE) 3.37.0.v20240206-1609, environment: Java 17.0.10 (Eclipse Adoptium)"
)
@Component
public class ReadingsMapperImpl implements ReadingsMapper {

    @Override
    public ReadingsDTO map(Readings readings) {
        if ( readings == null ) {
            return null;
        }

        ReadingsDTO readingsDTO = new ReadingsDTO();

        readingsDTO.setId( readings.getId() );
        readingsDTO.setVoltage1( readings.getVoltage1() );
        readingsDTO.setCurrent1( readings.getCurrent1() );
        readingsDTO.setPower1( readings.getPower1() );
        readingsDTO.setFrequency1( readings.getFrequency1() );
        readingsDTO.setVoltage2( readings.getVoltage2() );
        readingsDTO.setCurrent2( readings.getCurrent2() );
        readingsDTO.setPower2( readings.getPower2() );
        readingsDTO.setFrequency2( readings.getFrequency2() );
        readingsDTO.setVoltage3( readings.getVoltage3() );
        readingsDTO.setCurrent3( readings.getCurrent3() );
        readingsDTO.setPower3( readings.getPower3() );
        readingsDTO.setFrequency3( readings.getFrequency3() );
        readingsDTO.setSequence( readings.getSequence() );
        readingsDTO.setCreatedAt( readings.getCreatedAt() );
        readingsDTO.setDeviceId( readings.getDeviceId() );

        return readingsDTO;
    }

    @Override
    public List<ReadingsDTO> map(List<Readings> readingsList) {
        if ( readingsList == null ) {
            return null;
        }

        List<ReadingsDTO> list = new ArrayList<ReadingsDTO>( readingsList.size() );
        for ( Readings readings : readingsList ) {
            list.add( map( readings ) );
        }

        return list;
    }
}
