package com.esp32.coolp.entity;

import lombok.*;


import jakarta.persistence.*;
import java.sql.Timestamp;
import java.util.List;


@Data
@NoArgsConstructor
@AllArgsConstructor
@Entity
@Table(name = "device")
public class Device {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
     Long id;
    @Column(name = "name")
     String name;
    @Column(name = "created_at")
     Timestamp createdAt;

    @Transient
    @OneToMany(mappedBy = "readings", fetch = FetchType.LAZY)
    private List<Readings> readings;


}
