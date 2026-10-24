/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Build glue for the downstream (3.18) Samsung SCSC WLAN driver on a current
 * cfg80211 (the mac80211 backports of OpenWrt): names that were renamed.
 */
#ifndef SLSI_COMPAT_H
#define SLSI_COMPAT_H

#include <net/cfg80211.h>

/* enum ieee80211_band became enum nl80211_band in 4.7 */
#define ieee80211_band		nl80211_band
#define IEEE80211_BAND_2GHZ	NL80211_BAND_2GHZ
#define IEEE80211_BAND_5GHZ	NL80211_BAND_5GHZ
#define IEEE80211_NUM_BANDS	NUM_NL80211_BANDS

/* station_info.filled holds BIT_ULL(NL80211_STA_INFO_*) since 4.0 */
#define STATION_INFO_RX_BYTES		BIT_ULL(NL80211_STA_INFO_RX_BYTES)
#define STATION_INFO_TX_BYTES		BIT_ULL(NL80211_STA_INFO_TX_BYTES)
#define STATION_INFO_RX_PACKETS		BIT_ULL(NL80211_STA_INFO_RX_PACKETS)
#define STATION_INFO_TX_PACKETS		BIT_ULL(NL80211_STA_INFO_TX_PACKETS)
#define STATION_INFO_RX_DROP_MISC	BIT_ULL(NL80211_STA_INFO_RX_DROP_MISC)
#define STATION_INFO_TX_FAILED		BIT_ULL(NL80211_STA_INFO_TX_FAILED)
#define STATION_INFO_SIGNAL		BIT_ULL(NL80211_STA_INFO_SIGNAL)
#define STATION_INFO_BSS_PARAM		BIT_ULL(NL80211_STA_INFO_BSS_PARAM)
#define STATION_INFO_TX_BITRATE		BIT_ULL(NL80211_STA_INFO_TX_BITRATE)
#define STATION_INFO_RX_BITRATE		BIT_ULL(NL80211_STA_INFO_RX_BITRATE)
/* assoc_req_ies needs no flag any more, assoc_req_ies_len is enough */
#define STATION_INFO_ASSOC_REQ_IES	0

/* cfg80211_scan_done() takes a struct cfg80211_scan_info since 4.8 */
static inline void slsi_scan_done(struct cfg80211_scan_request *request, bool aborted)
{
	struct cfg80211_scan_info info = { .aborted = aborted };

	cfg80211_scan_done(request, &info);
}

/* from_timer() is gone since 6.16 */
#ifndef from_timer
#define from_timer(var, t, field)	timer_container_of(var, t, field)
#endif

#endif
