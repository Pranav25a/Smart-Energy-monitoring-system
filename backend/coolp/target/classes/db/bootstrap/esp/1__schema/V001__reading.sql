DROP TABLE IF EXISTS `readings`;
CREATE TABLE `readings`
(
    `id`                            bigint(20) NOT NULL AUTO_INCREMENT,
    `device_id`                     bigint(20) NOT NULL,
    `voltage1`                      DECIMAL(10, 3) DEFAULT NULL,
    `voltage2`                      DECIMAL(10, 3) DEFAULT NULL,
    `voltage3`                      DECIMAL(10, 3) DEFAULT NULL,
    `current1`                      DECIMAL(10, 3) DEFAULT NULL,
    `current2`                      DECIMAL(10, 3) DEFAULT NULL,
    `current3`                      DECIMAL(10, 3) DEFAULT NULL,
    `power1`                        DECIMAL(10, 3) DEFAULT NULL,
    `power2`                        DECIMAL(10, 3) DEFAULT NULL,
    `power3`                        DECIMAL(10, 3) DEFAULT NULL,
    `frequency1`                    DECIMAL(10, 3) DEFAULT NULL,
    `frequency2`                    DECIMAL(10, 3) DEFAULT NULL,
    `frequency3`                    DECIMAL(10, 3) DEFAULT NULL,
    `powerunit`                     DECIMAL(10, 4) DEFAULT NULL,
    `sequence`                      BIT DEFAULT NULL,
    `created_at`                    TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (`id`),
    FOREIGN KEY (`device_id`) REFERENCES `device` (`id`) ON DELETE CASCADE ON UPDATE NO ACTION
) ENGINE=InnoDB DEFAULT CHARSET=utf8 COLLATE utf8_unicode_ci;
