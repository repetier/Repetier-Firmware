#include "Repetier.h"

#if USB_HOST_SUPPORT == 1
#include "usbh_core.h"
#include "usbh_msc.h"

bool usbStorageAvailable = false;
USBH_HandleTypeDef hUsbHostFS;

/*
 * user callback definition
 */
extern "C" {
void DbgLog(char* msg) {
    Com::print(msg);
    Com::println();
    HAL::delayMilliseconds(20);
}
static void USBH_UserProcess(USBH_HandleTypeDef* phost, uint8_t id) {
    switch (id) {
    case HOST_USER_SELECT_CONFIGURATION:
        Com::printFLN(PSTR("UProc Select")); // TODO
        HAL::delayMilliseconds(20);          // TODO

        break;

    case HOST_USER_DISCONNECTION:
        Com::printFLN(PSTR("UProc Disc")); // TODO
        HAL::delayMilliseconds(20);        // TODO
        usbStorageAvailable = false;
        //Appli_state = APPLICATION_DISCONNECT;
        //Unmount_USB();
        break;

    case HOST_USER_CLASS_ACTIVE:
        Com::printFLN(PSTR("UProc act")); // TODO
        HAL::delayMilliseconds(20);       // TODO
        usbStorageAvailable = true;
        /* Appli_state = APPLICATION_READY;

        Mount_USB();

        Check_USB_Details(); // check space details

        Scan_USB("/"); // scan for files and directories

        Create_File("/ROOTFILE.txt");
        Write_File("/ROOTFILE.txt", "This data should be in root file\n");

        Create_Dir("/DIR1");
        Create_File("/DIR1/DIR1FILE.txt");
        Write_File("/DIR1/DIR1FILE.txt", "This data should be in DIR1 file\n");

        Create_Dir("/DIR2");
        Create_Dir("/DIR2/SUBDIR1");
        Create_File("/DIR2/SUBDIR1/DIR2FILE.txt");
        Write_File("/DIR2/SUBDIR1/DIR2FILE.txt", "This data should be in DIR2/SUBDIR1 file\n as i have nothing better to write/n so i just wrote this\n");

        Update_File("/ROOTFILE.txt", "This updated data must be in second line of Root File\n");
        */
        break;

    case HOST_USER_CONNECTION:
        // Appli_state = APPLICATION_START;
        break;

    default:
        break;
    }
}
}

static int hostUsbError = 998;

void initHostUSB() {
    hostUsbError = 999;
    if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_HS) != USBH_OK) {
        hostUsbError = 1;
        return;
    }
    if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK) {
        hostUsbError = 2;
        return;
        // Error_Handler();
    }
    if (USBH_Start(&hUsbHostFS) != USBH_OK) {
        hostUsbError = 3;
        return;
        // Error_Handler();
    }
    hostUsbError = 0;
}

RFUSBDrive::RFUSBDrive() {
    initHostUSB();
}

bool RFUSBDrive::driveAvailable() {
    if (hostUsbError) {
        return false;
    }
    USBH_Process(&hUsbHostFS);
    return usbStorageAvailable;
}

bool RFUSBDrive::init() {
    if (hostUsbError) {
        Com::printFLN(PSTR("USBInitError:"), hostUsbError);
        lastErrorCode = SD_CARD_ERROR_INIT_NOT_CALLED;
    }
    lastErrorCode = SD_CARD_ERROR_NONE;
    return true;
}
/** end use of device */
void RFUSBDrive::end() { }
/**
   * Check for FsBlockDevice busy.
   *
   * \return true if busy else false.
   */
bool RFUSBDrive::isBusy() {
    return false;
}
/**
   * Read a sector.
   *
   * \param[in] sector Logical sector to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
bool RFUSBDrive::readSector(uint32_t sector, uint8_t* dst) {
    MSC_LUNTypeDef info;

    if (USBH_MSC_Read(&hUsbHostFS, 0, sector, dst, 1) == USBH_OK) {
        return true;
    }
    USBH_MSC_GetLUNInfo(&hUsbHostFS, 0, &info);

    switch (info.sense.asc) {
    case SCSI_ASC_LOGICAL_UNIT_NOT_READY:
    case SCSI_ASC_MEDIUM_NOT_PRESENT:
    case SCSI_ASC_NOT_READY_TO_READY_CHANGE:
        Com::printFLN("USB Disk is not ready!");
        break;

    default:
        break;
    }
    return false;
}

/**
   * Read multiple sectors.
   *
   * \param[in] sector Logical sector to be read.
   * \param[in] ns Number of sectors to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
bool RFUSBDrive::readSectors(uint32_t sector, uint8_t* dst, size_t ns) {
    while (ns) {
        if (!readSector(sector, dst)) {
            return false;
        }
        sector++;
        dst += 512;
        ns--;
    }
    return true;
}

/** \return device size in sectors. */
uint32_t RFUSBDrive::sectorCount() {
    MSC_LUNTypeDef info;
    if (USBH_MSC_GetLUNInfo(&hUsbHostFS, 0, &info) == USBH_OK) {
        return info.capacity.block_nbr;
    }
    return 0;
    //csd_t csd;
    //return readCSD(&csd) ? csd.capacity() : 0;

    // return SDIO_GetBlocks();
}

/** End multi-sector transfer and go to idle state.
   * \return true for success or false for failure.
   */
bool RFUSBDrive::syncDevice() {
    return true; // read/write wait for end already
}

/**
   * Writes a sector.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
bool RFUSBDrive::writeSector(uint32_t sector, const uint8_t* src) {
    MSC_LUNTypeDef info;

    if (USBH_MSC_Write(&hUsbHostFS, 0, sector, (uint8_t*)src, 1) == USBH_OK) {
        return true;
    }
    USBH_MSC_GetLUNInfo(&hUsbHostFS, 0, &info);

    switch (info.sense.asc) {
    case SCSI_ASC_WRITE_PROTECTED:
        Com::printFLN("USB Disk is Write protected!");
        break;

    case SCSI_ASC_LOGICAL_UNIT_NOT_READY:
    case SCSI_ASC_MEDIUM_NOT_PRESENT:
    case SCSI_ASC_NOT_READY_TO_READY_CHANGE:
        Com::printFLN("USB Disk is not ready!");
        break;

    default:
        break;
    }
    return false;
}

/**
   * Write multiple sectors.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] ns Number of sectors to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
bool RFUSBDrive::writeSectors(uint32_t sector, const uint8_t* src, size_t ns) {
    while (ns) {
        if (!writeSector(sector, src)) {
            return false;
        }
        sector++;
        ns--;
        src += 512;
    }
    return true;
}

/** CMD6 Switch mode: Check Function Set Function.
   * \param[in] arg CMD6 argument.
   * \param[out] status return status data.
   *
   * \return true for success or false for failure.
   */
bool RFUSBDrive::cardCMD6(uint32_t arg, uint8_t* status) {
    return false; // unused externally
}

/** Erase a range of sectors.
   *
   * \param[in] firstSector The address of the first sector in the range.
   * \param[in] lastSector The address of the last sector in the range.
   *
   * \return true for success or false for failure.
   */
bool RFUSBDrive::erase(uint32_t firstSector, uint32_t lastSector) {
    return false; // not used
}
/** \return error code. */
uint8_t RFUSBDrive::errorCode() const {
    return lastErrorCode;
}
/** \return error data. */
uint32_t RFUSBDrive::errorData() const {
    return lastErrorLine;
}
/** \return false by default */
bool RFUSBDrive::hasDedicatedSpi() { return false; }
/** \return false by default */
bool RFUSBDrive::isDedicatedSpi() { return false; }
/** Set SPI sharing state
   * \param[in] value desired state.
   * \return false by default.
   */
bool RFUSBDrive::setDedicatedSpi(bool value) {
    (void)value;
    return false;
}
/**
   * Read a card's CID register.
   *
   * \param[out] cid pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
bool RFUSBDrive::readCID(cid_t* cid) {
    return false; // unused by FatLib
}
/**
   * Read a card's CSD register.
   *
   * \param[out] csd pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
bool RFUSBDrive::readCSD(csd_t* csd) {
    // memcpy(csd, hsd.CSD, 16);
    return true;
}
/** Read OCR register.
   *
   * \param[out] ocr Value of OCR register.
   * \return true for success or false for failure.
   */
bool RFUSBDrive::readOCR(uint32_t* ocr) {
    return false; // unused
}
/** Read SCR register.
   *
   * \param[out] scr Value of SCR register.
   * \return true for success or false for failure.
   */
bool RFUSBDrive::readSCR(scr_t* scr) {
    return false; // unused
}
/** \return card status. */
uint32_t RFUSBDrive::status() { return 0XFFFFFFFF; }
/** Return the card type: SD V1, SD V2 or SDHC/SDXC
   * \return 0 - SD V1, 1 - SD V2, or 3 - SDHC/SDXC.
   */
uint8_t RFUSBDrive::type() const {
    return 3;
}
/** Write one data sector in a multiple sector write sequence.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */

bool RFUSBDrive::writeData(const uint8_t* src) {
    return false; // unused
}
/** Start a write multiple sectors sequence.
   *
   * \param[in] sector Address of first sector in sequence.
   *
   * \return true for success or false for failure.
   */
bool RFUSBDrive::writeStart(uint32_t sector) {
    return false; // unused
}
/** End a write multiple sectors sequence.
   * \return true for success or false for failure.
   */
bool RFUSBDrive::writeStop() {
    return false; // unused
}

#endif
