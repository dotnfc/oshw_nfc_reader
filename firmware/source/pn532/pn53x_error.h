#ifndef _PN53X_ERROR_H
#define _PN53X_ERROR_H


#define NFC_SUCCESS              0 /**< Success (no error) */
#define NFC_EIO				    -1 /**< Input / output error, device may not be usable anymore without re-open it */
#define NFC_EINVARG			    -2 /**< Invalid argument(s) */
#define NFC_EDEVNOTSUPP         -3  /**< Operation not supported by device */
#define NFC_ENOTSUCHDEV			-4
#define NFC_EOVFLOW             -5 /**< Buffer overflow */
#define NFC_ETIMEOUT            -6 /**< Operation timed out */
#define NFC_EOPABORTED          -7 /**< Operation aborted (by user) */
#define NFC_ENOTIMPL            -8 /**< Not (yet) implemented */
#define NFC_EBUS_TRANS          -9 /**< bus trans failed */
#define NFC_ETGRELEASED         -10 /**< Target released */
#define NFC_ERFTRANS            -20 /**< Error while RF transmission */
#define NFC_EMFCAUTHFAIL        -30 /**< MIFARE Classic: authentication failed */
#define NFC_ESOFT               -80 /**< Software error (allocation, file/pipe creation, etc.) */
#define NFC_ECHIP               -90 /**< Device's internal chip error */
#define NFC_ENO_TGT            -100 /**< Target not found */

#endif // _PN53X_ERROR_H
