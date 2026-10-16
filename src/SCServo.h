/*
 * SCServo.h
 * FT serial servo public interface layer.
 * Date: 2025.9.27
 * Author:
 */

#ifndef _SCSERVO_H
#define _SCSERVO_H

/**
 * @brief Convenience umbrella header that exposes the supported FT servo families.
 *
 * The library provides protocol-specific driver classes for the SCSCL, SMS_STS,
 * and HLSCL product lines, and this header makes them available to applications
 * through a single include path.
 */
#include "SCSCL.h"
#include "SMS_STS.h"
#include "HLSCL.h"

#endif