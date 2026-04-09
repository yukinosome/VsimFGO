////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
// The names of messages are ROS messages, not Oem7 messages. These may be identical, but not necessarily.
// More than one Oem7 message may be used to generate the same ROS message
//

#include "novatel_oem7_driver/oem7_ros_messages.hpp"

#include "novatel_oem7_driver/oem7_message_ids.h"
#include "novatel_oem7_driver/oem7_messages.h"
#include "novatel_oem7_driver/oem7_message_util.hpp"


#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/bestutm.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include "novatel_oem7_msgs/msg/insconfig.hpp"
#include "novatel_oem7_msgs/msg/insstdev.hpp"
#include "novatel_oem7_msgs/msg/corrimu.hpp"
#include "novatel_oem7_msgs/msg/rxstatus.hpp"
#include "novatel_oem7_msgs/msg/time.hpp"

#include "novatel_oem7_msgs/msg/range.hpp"
#include "novatel_oem7_msgs/msg/clockmodel.hpp"
#include "novatel_oem7_msgs/msg/ionutc.hpp"
#include "novatel_oem7_msgs/msg/gpsephem.hpp"
#include "novatel_oem7_msgs/msg/galclock.hpp"
#include "novatel_oem7_msgs/msg/galiono.hpp"
#include "novatel_oem7_msgs/msg/galfnavephemeris.hpp"
#include "novatel_oem7_msgs/msg/galinavephemeris.hpp"
#include "novatel_oem7_msgs/msg/dualantennaheading.hpp"
#include "novatel_oem7_msgs/msg/range_data.hpp"
#include "novatel_oem7_msgs/msg/sbasalmanac.hpp"
#include <novatel_oem7_msgs/msg/navicsysclock.hpp>


#define arr_size(_arr_) (sizeof(_arr_) / sizeof(_arr_[0]))

namespace novatel_oem7_driver
{

/*
 * Populates Oem7header from raw message
 */
void
SetOem7Header(
    const Oem7RawMessageIf::ConstPtr& msg, ///< in: raw message
    const std::string& name, ///< message name
    novatel_oem7_msgs::msg::Oem7Header::Type& oem7_hdr ///< header to populate
    )
{
  getOem7Header(msg, oem7_hdr);
  oem7_hdr.message_name = name;
}

/*
 * Populates Oem7header from a 'short' raw message
 */
void
SetOem7ShortHeader(
    const Oem7RawMessageIf::ConstPtr& msg, ///< in: short raw message
    const std::string& name, ///< message name
    novatel_oem7_msgs::msg::Oem7Header::Type& oem7_hdr ///< header to populate
    )
{
  getOem7ShortHeader(msg, oem7_hdr);
  oem7_hdr.message_name = name;
}

// Template specializations from specific messages
template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::NAVICSYSCLOCK>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::NAVICSYSCLOCK>& navicsysclock)
{
  assert(msg->getMessageId() == NAVICSYSCLOCK_OEM7_MSGID);
  const auto* mem = reinterpret_cast<const NAVICSYSCLOCKMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  navicsysclock = std::make_shared<novatel_oem7_msgs::msg::NAVICSYSCLOCK>();
  navicsysclock->prn = mem->PRN;
  navicsysclock->a0_utc = mem->A0_utc;
  navicsysclock->a1_utc = mem->A1_utc;
  navicsysclock->a2_utc = mem->A2_utc;
  navicsysclock->delta_t_ls = mem->delta_T_LS;
  navicsysclock->delta_t_lsf = mem->delta_T_LSF;
  navicsysclock->t_outc = mem->T_outc;
  navicsysclock->wn_outc = mem->WN_outc;
  navicsysclock->wn_lsf = mem->WN_LSF;
  navicsysclock->dn = mem->DN;
  navicsysclock->gnss_id = mem->GNSSID;
  navicsysclock->a0 = mem->A0;
  navicsysclock->a1 = mem->A1;
  navicsysclock->a2 = mem->A2;
  navicsysclock->tot = mem->Tot;
  navicsysclock->wnot = mem->WNot;
  navicsysclock->spare = mem->Spare;

  static const std::string name = "NAVICSYSCLOCK";
  SetOem7Header(msg, name, navicsysclock->nov_header);
}


template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::HEADING2>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::HEADING2>& heading2)
{
  assert(msg->getMessageId() == HEADING2_OEM7_MSGID);

  const auto* mem = reinterpret_cast<const HEADING2Mem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  heading2.reset(new novatel_oem7_msgs::msg::HEADING2);

  heading2->sol_status.status     = mem->sol_status;
  heading2->pos_type.type         = mem->pos_type;
  heading2->length                = mem->length;
  heading2->heading               = mem->heading;
  heading2->pitch                 = mem->pitch;
  heading2->reserved              = mem->reserved;
  heading2->heading_stdev         = mem->heading_stdev;
  heading2->pitch_stdev           = mem->pitch_stdev;
  std::copy(std::begin(mem->rover_stn_id),  std::end(mem->rover_stn_id),  std::begin(heading2->rover_stn_id));
  std::copy(std::begin(mem->master_stn_id), std::end(mem->master_stn_id), std::begin(heading2->master_stn_id));
  heading2->num_sv_tracked          = mem->num_sv_tracked;
  heading2->num_sv_in_sol           = mem->num_sv_in_sol;
  heading2->num_sv_obs              = mem->num_sv_obs;
  heading2->num_sv_multi            = mem->num_sv_multi;
  heading2->sol_source.source       = mem->sol_source;
  heading2->ext_sol_status.status   = mem->ext_sol_status;
  heading2->galileo_beidou_sig_mask = mem->galileo_beidou_sig_mask;
  heading2->gps_glonass_sig_mask    = mem->gps_glonass_sig_mask;

  static const std::string name = "HEADING2";
  SetOem7Header(msg, name, heading2->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::BESTPOS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::BESTPOS>& bestpos)
{
  assert(msg->getMessageId() == BESTPOS_OEM7_MSGID);

  const auto* bp = reinterpret_cast<const BESTPOSMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  bestpos.reset(new novatel_oem7_msgs::msg::BESTPOS);

  bestpos->sol_status.status      = bp->sol_stat;
  bestpos->pos_type.type          = bp->pos_type;
  bestpos->lat                    = bp->lat;
  bestpos->lon                    = bp->lon;
  bestpos->hgt                    = bp->hgt;
  bestpos->undulation             = bp->undulation;
  bestpos->datum_id               = bp->datum_id;
  bestpos->lat_stdev              = bp->lat_stdev;
  bestpos->lon_stdev              = bp->lon_stdev;
  bestpos->hgt_stdev              = bp->hgt_stdev;
  std::copy(std::begin(bp->stn_id), std::end(bp->stn_id), std::begin(bestpos->stn_id));
  bestpos->diff_age               = bp->diff_age;
  bestpos->sol_age                = bp->sol_age;
  bestpos->num_svs                = bp->num_svs;
  bestpos->num_sol_svs            = bp->num_sol_svs;
  bestpos->num_sol_l1_svs         = bp->num_sol_l1_svs;
  bestpos->num_sol_multi_svs      = bp->num_sol_multi_svs;
  bestpos->reserved               = bp->reserved;
  bestpos->ext_sol_stat.status    = bp->ext_sol_stat;
  bestpos->galileo_beidou_sig_mask= bp->galileo_beidou_sig_mask;
  bestpos->gps_glonass_sig_mask   = bp->gps_glonass_sig_mask;

  static const std::string name = "BESTPOS";
  SetOem7Header(msg, name, bestpos->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::BESTVEL>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::BESTVEL>& bestvel)
{
  assert(msg->getMessageId() == BESTVEL_OEM7_MSGID);

  const auto* bv = reinterpret_cast<const BESTVELMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  bestvel.reset(new novatel_oem7_msgs::msg::BESTVEL);

  bestvel->sol_status.status = bv->sol_stat;
  bestvel->vel_type.type     = bv->vel_type;
  bestvel->latency           = bv->latency;
  bestvel->diff_age          = bv->diff_age;
  bestvel->hor_speed         = bv->hor_speed;
  bestvel->trk_gnd           = bv->track_gnd;
  bestvel->ver_speed         = bv->ver_speed;
  bestvel->reserved          = bv->reserved;

  static const std::string name = "BESTVEL";
  SetOem7Header(msg, name, bestvel->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::BESTUTM>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::BESTUTM>& bestutm)
{
    assert(msg->getMessageId() == BESTUTM_OEM7_MSGID);

    const auto* mem = reinterpret_cast<const BESTUTMMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
    bestutm.reset(new novatel_oem7_msgs::msg::BESTUTM);

    bestutm->pos_type.type          = mem->pos_type;;
    bestutm->lon_zone_number        = mem->lon_zone_number;
    bestutm->lat_zone_letter        = mem->lat_zone_letter;
    bestutm->northing               = mem->northing;
    bestutm->easting                = mem->easting;
    bestutm->height                 = mem->height;
    bestutm->undulation             = mem->undulation;
    bestutm->datum_id               = mem->datum_id;
    bestutm->northing_stddev        = mem->northing_stddev;
    bestutm->easting_stddev         = mem->easting_stddev;
    bestutm->height_stddev          = mem->height_stddev;
    std::copy(std::begin(mem->stn_id), std::end(mem->stn_id), std::begin(bestutm->stn_id));
    bestutm->diff_age               = mem->diff_age;
    bestutm->sol_age                = mem->sol_age;
    bestutm->num_svs                = mem->num_svs;
    bestutm->num_sol_svs            = mem->num_sol_svs;
    bestutm->num_sol_ggl1_svs       = mem->num_sol_ggl1_svs;
    bestutm->num_sol_multi_svs      = mem->num_sol_multi_svs;
    bestutm->reserved               = mem->reserved;
    bestutm->ext_sol_stat.status    = mem->ext_sol_stat;
    bestutm->galileo_beidou_sig_mask= mem->galileo_beidou_sig_mask;
    bestutm->gps_glonass_sig_mask   = mem->gps_glonass_sig_mask;

    static const std::string name = "BESTUTM";
    SetOem7Header(msg, name, bestutm->nov_header);
  }

/*
 * new messages
 */

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::SBASALMANAC>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::SBASALMANAC>& sbasalmanac)
{
  assert(msg->getMessageId() == SBASALMANAC_OEM7_MSGID);

  const auto* mem = reinterpret_cast<const SBASALMANACMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  sbasalmanac.reset(new novatel_oem7_msgs::msg::SBASALMANAC);

  sbasalmanac->x_ecef                = mem->X_ecef;
  sbasalmanac->y_ecef                = mem->Y_ecef;
  sbasalmanac->z_ecef                = mem->Z_ecef;
  sbasalmanac->x_vel                 = mem->X_vel;
  sbasalmanac->y_vel                 = mem->Y_vel;
  sbasalmanac->z_vel                 = mem->Z_vel;
  sbasalmanac->data_id               = mem->data_id;
  sbasalmanac->satellite_health      = mem->satellite_health;
  sbasalmanac->system_variant        = mem->system_variant;
  sbasalmanac->time                  = mem->time;

  static const std::string name = "SBASALMANAC";
  SetOem7Header(msg, name, sbasalmanac->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::CLOCKMODEL>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::CLOCKMODEL>& clockmodel_msg
)
{
  assert(msg->getMessageId() == CLOCKMODEL_OEM7_MSGID);
  const auto* clockmodel = reinterpret_cast<const CLOCKMODELMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  clockmodel_msg.reset(new novatel_oem7_msgs::msg::CLOCKMODEL);

  clockmodel_msg->sol_status.status = clockmodel->status;
  clockmodel_msg->reject_count = clockmodel->reject_count;
  clockmodel_msg->propagation_time = clockmodel->propagation_time;
  clockmodel_msg->update_time = clockmodel->update_time;
  clockmodel_msg->bias = clockmodel->bias;
  clockmodel_msg->rate = clockmodel->rate;
  clockmodel_msg->reserved1 = clockmodel->reserved1;
  clockmodel_msg->bias_variance = clockmodel->bias_variance;
  clockmodel_msg->covariance = clockmodel->covariance;
  clockmodel_msg->reserved2 = clockmodel->reserved2;
  clockmodel_msg->reserved3 = clockmodel->reserved3;
  clockmodel_msg->reserved4 = clockmodel->reserved4;
  clockmodel_msg->rate_variance = clockmodel->rate_variance;
  clockmodel_msg->reserved5 = clockmodel->reserved5;
  clockmodel_msg->reserved6 = clockmodel->reserved6;
  clockmodel_msg->reserved7 = clockmodel->reserved7;
  clockmodel_msg->instantaneous_bias = clockmodel->instantaneous_bias;
  clockmodel_msg->instantaneous_rate = clockmodel->instantaneous_rate;
  clockmodel_msg->reserved8 = clockmodel->reserved8;

  static const std::string name = "CLOCKMODEL";
  SetOem7Header(msg, name, clockmodel_msg->nov_header);
}

template<>
void MakeROSMessage<novatel_oem7_msgs::msg::DUALANTENNAHEADING>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::DUALANTENNAHEADING>& dualantenna_heading_msg)
{
  assert(msg->getMessageId() == DUALANTENNAHEADING_OEM7_MSGID);
  const auto* heading = reinterpret_cast<const DUALANTENNAHEADINGMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  dualantenna_heading_msg.reset(new novatel_oem7_msgs::msg::DUALANTENNAHEADING);

  dualantenna_heading_msg->sol_status.status = heading->sol_status;
  dualantenna_heading_msg->pos_type.type = heading->pos_type;
  dualantenna_heading_msg->length = heading->length;
  dualantenna_heading_msg->heading = heading->heading;
  dualantenna_heading_msg->pitch = heading->pitch;
  dualantenna_heading_msg->reserved = heading->reserved;
  dualantenna_heading_msg->heading_std_dev = heading->heading_std_dev;
  dualantenna_heading_msg->pitch_std_dev = heading->pitch_std_dev;
  dualantenna_heading_msg->rover_stn_id.assign(heading->rover_stn_id, arr_size(heading->rover_stn_id));
  dualantenna_heading_msg->num_sv_tracked = heading->num_sv_tracked;
  dualantenna_heading_msg->num_sv_in_sol = heading->num_sv_in_sol;
  dualantenna_heading_msg->num_sv_obs = heading->num_sv_obs;
  dualantenna_heading_msg->num_sv_multi = heading->num_sv_multi;
  dualantenna_heading_msg->sol_source = heading->sol_source;
  dualantenna_heading_msg->ext_sol_status = heading->ext_sol_status;
  dualantenna_heading_msg->galileo_beidou_sig_mask = heading->galileo_beidou_sig_mask;
  dualantenna_heading_msg->gps_glonass_sig_mask = heading->gps_glonass_sig_mask;

  static const std::string name = "DUALANTENNAHEADING";
  SetOem7Header(msg, name, dualantenna_heading_msg->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::GALCLOCK>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::GALCLOCK>& galclock_msg)
{
  assert(msg->getMessageId() == GALCLOCK_OEM7_MSGID);
  galclock_msg.reset(new novatel_oem7_msgs::msg::GALCLOCK);

  const auto* galclock = reinterpret_cast<const GALCLOCKMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));

  galclock_msg->a0 = galclock->A0;
  galclock_msg->a1 = galclock->A1;
  galclock_msg->delta_tls = galclock->DeltaTls;
  galclock_msg->tot = galclock->Tot;
  galclock_msg->wnt = galclock->WNt;
  galclock_msg->wnlsf = galclock->WNlsf;
  galclock_msg->dn = galclock->DN;
  galclock_msg->delta_tlsf = galclock->DeltaTlsf;
  galclock_msg->a0g = galclock->A0g;
  galclock_msg->a1g = galclock->A1g;
  galclock_msg->t0g = galclock->T0g;
  galclock_msg->wn0g = galclock->WN0g;

  static const std::string name = "GALCLOCK";
  SetOem7Header(msg, name, galclock_msg->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::GALIONO>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::GALIONO>& galiono_msg)
{
  assert(msg->getMessageId() == GALIONO_OEM7_MSGID);
  galiono_msg.reset(new novatel_oem7_msgs::msg::GALIONO);
  const auto* galiono = reinterpret_cast<const GALIONOMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));

  galiono_msg->ai0 = galiono->Ai0;
  galiono_msg->ai1 = galiono->Ai1;
  galiono_msg->ai2 = galiono->Ai2;
  galiono_msg->sf1 = galiono->SF1;
  galiono_msg->sf2 = galiono->SF2;
  galiono_msg->sf3 = galiono->SF3;
  galiono_msg->sf4 = galiono->SF4;
  galiono_msg->sf5 = galiono->SF5;

  static const std::string name = "GALIONO";
  SetOem7Header(msg, name, galiono_msg->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::IONUTC>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::IONUTC>& ionutc_msg)
{
  assert(msg->getMessageId() == IONUTC_OEM7_MSGID);
  ionutc_msg.reset(new novatel_oem7_msgs::msg::IONUTC);
  const auto* ionutc = reinterpret_cast<const IONUTCMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));

  ionutc_msg->a0 = ionutc->a0;
  ionutc_msg->a1 = ionutc->a1;
  ionutc_msg->a2 = ionutc->a2;
  ionutc_msg->a3 = ionutc->a3;
  ionutc_msg->b0 = ionutc->b0;
  ionutc_msg->b1 = ionutc->b1;
  ionutc_msg->b2 = ionutc->b2;
  ionutc_msg->b3 = ionutc->b3;
  ionutc_msg->utc_wn = ionutc->utc_wn;
  ionutc_msg->tot = ionutc->tot;
  ionutc_msg->capital_a0 = ionutc->A0;
  ionutc_msg->capital_a1 = ionutc->A1;
  ionutc_msg->wn_lsf = ionutc->wn_lsf;
  ionutc_msg->dn = ionutc->dn;
  ionutc_msg->delta_tls = ionutc->deltaT_ls;
  ionutc_msg->delta_tlsg = ionutc->deltaT_lsf;
  ionutc_msg->reserved = ionutc->reserved;

  static const std::string name = "IONUTC";
  SetOem7Header(msg, name, ionutc_msg->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::RANGE>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::RANGE>& ranges)
{
  assert(msg->getMessageId() == RANGE_OEM7_MSGID);
  ranges.reset(new novatel_oem7_msgs::msg::RANGE);

  const auto* range_header = reinterpret_cast<const RANGEHeaderMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));

  ranges->number_obs = range_header->number_obs;

  // we check the the rest data length are correct for RANGE_DATA_LEN * number_obs
  assert((msg->getMessageDataLength() - RANGE_DATA_OFFSET) / RANGE_DATA_LEN == ranges->number_obs );

  for(size_t i = 0; i < ranges->number_obs; i++)
  {
    const auto* range_data = reinterpret_cast<const RANGEDATAMem*>(msg->getMessageData(RANGE_DATA_OFFSET + i * RANGE_DATA_LEN));
    novatel_oem7_msgs::msg::RANGEData  range_msg;

    range_msg.prn = range_data->PRN;
    range_msg.glofreq = range_data->glofreq;
    range_msg.psr = range_data->psr;
    range_msg.psr_sigma = range_data->psr_sigma;
    range_msg.adr = range_data->adr;
    range_msg.adr_sigma = range_data->adr_sigma;
    range_msg.dopp = range_data->dopp;
    range_msg.cn0 = range_data->cn0;
    range_msg.locktime = range_data->locktime;
    range_msg.ch_tr_status = range_data->ch_tr_status;
    ranges->ranges.emplace_back(range_msg);
  }

  static const std::string name = "RANGE";
  SetOem7Header(msg, name, ranges->nov_header);

  if(ranges->nov_header.message_type & 0x01)
  {
    ranges->nov_header.message_name = "RANGE_AUX";
  }

}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::GALFNAVEPHEMERIS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::GALFNAVEPHEMERIS>& galFnavEphemeris_msg)
{
  assert(msg->getMessageId() == GALFNAVEPHEMERIS_OEM7_MSGID);
  galFnavEphemeris_msg.reset(new novatel_oem7_msgs::msg::GALFNAVEPHEMERIS);

  static const std::string name = "GALFNAVEPHEMERIS";
  const auto* galFnavEphemeris = reinterpret_cast<const GALFNAVEPHEMERISMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  galFnavEphemeris_msg->sat_id = galFnavEphemeris->SatId;
  galFnavEphemeris_msg->e5a_health = galFnavEphemeris->E5aHealth;
  galFnavEphemeris_msg->e5a_dvs = galFnavEphemeris->E5aDVS;
  galFnavEphemeris_msg->reserved1 = galFnavEphemeris->reserved1;
  galFnavEphemeris_msg->reserved2 = galFnavEphemeris->reserved2;
  galFnavEphemeris_msg->iod_nav = galFnavEphemeris->IODnav;
  galFnavEphemeris_msg->sisa_index = galFnavEphemeris->SISA_Index;
  galFnavEphemeris_msg->reserved3 = galFnavEphemeris->reserved3;
  galFnavEphemeris_msg->t0e = galFnavEphemeris->T0e;
  galFnavEphemeris_msg->t0c = galFnavEphemeris->T0c;
  galFnavEphemeris_msg->m0 = galFnavEphemeris->M0;
  galFnavEphemeris_msg->delta_n = galFnavEphemeris->DeltaN;
  galFnavEphemeris_msg->ecc = galFnavEphemeris->Ecc;
  galFnavEphemeris_msg->root_a = galFnavEphemeris->RootA;
  galFnavEphemeris_msg->i0 = galFnavEphemeris->I0;
  galFnavEphemeris_msg->i_dot = galFnavEphemeris->IDot;
  galFnavEphemeris_msg->omega0 = galFnavEphemeris->Omega0;
  galFnavEphemeris_msg->omega = galFnavEphemeris->Omega;
  galFnavEphemeris_msg->omega_dot = galFnavEphemeris->OmegaDot;
  galFnavEphemeris_msg->cuc = galFnavEphemeris->Cuc;
  galFnavEphemeris_msg->cus = galFnavEphemeris->Cus;
  galFnavEphemeris_msg->crc = galFnavEphemeris->Crc;
  galFnavEphemeris_msg->crs = galFnavEphemeris->Crs;
  galFnavEphemeris_msg->cic = galFnavEphemeris->Cic;
  galFnavEphemeris_msg->cis = galFnavEphemeris->Cis;
  galFnavEphemeris_msg->af0 = galFnavEphemeris->Af0;
  galFnavEphemeris_msg->af1 = galFnavEphemeris->Af1;
  galFnavEphemeris_msg->af2 = galFnavEphemeris->Af2;
  galFnavEphemeris_msg->e1e5a_bgd = galFnavEphemeris->E1E5aBGD;

  SetOem7Header(msg, name, galFnavEphemeris_msg->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::GALINAVEPHEMERIS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::GALINAVEPHEMERIS>& galInavEphemeris_msg)
{
  assert(msg->getMessageId() == GALINAVEPHEMERIS_OEM7_MSGID);
  galInavEphemeris_msg.reset(new novatel_oem7_msgs::msg::GALINAVEPHEMERIS);

  static const std::string name = "GALINAVEPHEMERIS";

  const auto* galInavEphemeris = reinterpret_cast<const GALINAVEPHEMERISMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));

  galInavEphemeris_msg->sat_id = galInavEphemeris->SatId;
  galInavEphemeris_msg->e5b_health = galInavEphemeris->E5bHealth;
  galInavEphemeris_msg->e5b_dvs = galInavEphemeris->E5bDVS;
  galInavEphemeris_msg->reserved1 = galInavEphemeris->reserved1;
  galInavEphemeris_msg->reserved2 = galInavEphemeris->reserved2;
  galInavEphemeris_msg->e1b_health = galInavEphemeris->E1bHealth;
  galInavEphemeris_msg->e1b_dvs = galInavEphemeris->E1bDVS;
  galInavEphemeris_msg->reserved3 = galInavEphemeris->reserved3;
  galInavEphemeris_msg->reserved4 = galInavEphemeris->reserved4;
  galInavEphemeris_msg->io_dnav = galInavEphemeris->IODnav;
  galInavEphemeris_msg->sisa_index = galInavEphemeris->SISA_Index;
  galInavEphemeris_msg->inav_source = galInavEphemeris->INAV_Source;
  galInavEphemeris_msg->t0e = galInavEphemeris->T0e;
  galInavEphemeris_msg->t0c = galInavEphemeris->T0c;
  galInavEphemeris_msg->m0 = galInavEphemeris->M0;
  galInavEphemeris_msg->delta_n = galInavEphemeris->DeltaN;
  galInavEphemeris_msg->ecc = galInavEphemeris->Ecc;
  galInavEphemeris_msg->root_a = galInavEphemeris->RootA;
  galInavEphemeris_msg->i0 = galInavEphemeris->I0;
  galInavEphemeris_msg->i_dot = galInavEphemeris->IDot;
  galInavEphemeris_msg->omega0 = galInavEphemeris->Omega0;
  galInavEphemeris_msg->omega = galInavEphemeris->Omega;
  galInavEphemeris_msg->omega_dot = galInavEphemeris->OmegaDot;
  galInavEphemeris_msg->cuc = galInavEphemeris->Cuc;
  galInavEphemeris_msg->cus = galInavEphemeris->Cus;
  galInavEphemeris_msg->crc = galInavEphemeris->Crc;
  galInavEphemeris_msg->crs = galInavEphemeris->Crs;
  galInavEphemeris_msg->cic = galInavEphemeris->Cic;
  galInavEphemeris_msg->cis = galInavEphemeris->Cis;
  galInavEphemeris_msg->af0 = galInavEphemeris->Af0;
  galInavEphemeris_msg->af1 = galInavEphemeris->Af1;
  galInavEphemeris_msg->af2 = galInavEphemeris->Af2;
  galInavEphemeris_msg->e1e5a_bgd = galInavEphemeris->E1E5aBGD;
  galInavEphemeris_msg->e1e5b_bgd = galInavEphemeris->E1E5bBGD;
  SetOem7Header(msg, name, galInavEphemeris_msg->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::GPSEPHEM>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::GPSEPHEM>& gpsephem_msg)
{
  assert(msg->getMessageId() == GPSEPHEM_OEM7_MSGID);
  gpsephem_msg.reset(new novatel_oem7_msgs::msg::GPSEPHEM);
  static const std::string name = "GPSEPHEM";
  const auto* gpsephem = reinterpret_cast<const GPSEPHEMMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  gpsephem_msg->prn = gpsephem->PRN;
  gpsephem_msg->tow = gpsephem->tow;
  gpsephem_msg->health = gpsephem->health;
  gpsephem_msg->iode1 = gpsephem->IODE1;
  gpsephem_msg->iode2 = gpsephem->IODE2;
  gpsephem_msg->week = gpsephem->week;
  gpsephem_msg->z_week = gpsephem->z_week;
  gpsephem_msg->t0e = gpsephem->T0e;
  gpsephem_msg->a = gpsephem->A;
  gpsephem_msg->delta_n = gpsephem->deltaN;
  gpsephem_msg->m0 = gpsephem->M0;
  gpsephem_msg->ecc = gpsephem->ecc;
  gpsephem_msg->omega = gpsephem->omega;
  gpsephem_msg->cuc = gpsephem->cuc;
  gpsephem_msg->cus = gpsephem->cus;
  gpsephem_msg->crc = gpsephem->crc;
  gpsephem_msg->crs = gpsephem->crs;
  gpsephem_msg->cic = gpsephem->cic;
  gpsephem_msg->cis = gpsephem->cis;
  gpsephem_msg->i0 = gpsephem->I0;
  gpsephem_msg->i_dot = gpsephem->IDot;
  gpsephem_msg->omega0 = gpsephem->omega0;
  gpsephem_msg->omega_dot = gpsephem->omegaDot;
  gpsephem_msg->iodc = gpsephem->iodc;
  gpsephem_msg->toc = gpsephem->toc;
  gpsephem_msg->tgd = gpsephem->tgd;
  gpsephem_msg->af0 = gpsephem->af0;
  gpsephem_msg->af1 = gpsephem->af1;
  gpsephem_msg->af2 = gpsephem->af2;
  gpsephem_msg->a_s = gpsephem->AS;
  gpsephem_msg->n = gpsephem->N;
  gpsephem_msg->ura = gpsephem->URA;
  SetOem7Header(msg, name, gpsephem_msg->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::INSPVA>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::INSPVA>& pva)
{
  assert(msg->getMessageId() == INSPVAS_OEM7_MSGID);

  const auto* pvamem = reinterpret_cast<const INSPVASmem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
  pva.reset(new novatel_oem7_msgs::msg::INSPVA);

  pva->latitude        =     pvamem->latitude;
  pva->longitude       =     pvamem->longitude;
  pva->height          =     pvamem->height;
  pva->north_velocity  =     pvamem->north_velocity;
  pva->east_velocity   =     pvamem->east_velocity;
  pva->up_velocity     =     pvamem->up_velocity;
  pva->roll            =     pvamem->roll;
  pva->pitch           =     pvamem->pitch;
  pva->azimuth         =     pvamem->azimuth;
  pva->status.status   =     pvamem->status;

  static const std::string name = "INSPVA";
  SetOem7ShortHeader(msg, name, pva->nov_header);
}


template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::INSCONFIG>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::INSCONFIG>& insconfig)
{
  assert(msg->getMessageId()== INSCONFIG_OEM7_MSGID);

  const auto* insconfigmem =
      reinterpret_cast<const INSCONFIG_FixedMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  insconfig.reset(new novatel_oem7_msgs::msg::INSCONFIG);

  insconfig->imu_type                         = insconfigmem->imu_type;
  insconfig->mapping                          = insconfigmem->mapping;
  insconfig->initial_alignment_velocity       = insconfigmem->initial_alignment_velocity;
  insconfig->heave_window                     = insconfigmem->heave_window;
  insconfig->profile                          = insconfigmem->profile;

  std::copy(
      insconfigmem->enabled_updates,
      insconfigmem->enabled_updates + arr_size(insconfigmem->enabled_updates),
      insconfig->enabled_updates.begin());

  insconfig->alignment_mode.mode              = insconfigmem->alignment_mode;
  insconfig->relative_ins_output_frame.frame  = insconfigmem->relative_ins_output_frame;
  insconfig->relative_ins_output_direction   = insconfigmem->relative_ins_output_direction;

  std::copy(
      insconfigmem->ins_receiver_status,
      insconfigmem->ins_receiver_status + arr_size(insconfigmem->ins_receiver_status),
      insconfig->ins_receiver_status.status.begin());

  insconfig->ins_seed_enabled                 = insconfigmem->ins_seed_enabled;
  insconfig->ins_seed_validation              = insconfigmem->ins_seed_validation;
  insconfig->reserved_1 = insconfigmem->reserved_1;
  insconfig->reserved_2 = insconfigmem->reserved_2;
  insconfig->reserved_3 = insconfigmem->reserved_3;
  insconfig->reserved_4 = insconfigmem->reserved_4;
  insconfig->reserved_5 = insconfigmem->reserved_5;
  insconfig->reserved_6 = insconfigmem->reserved_6;
  insconfig->reserved_7 = insconfigmem->reserved_7;

  insconfig->translations.reserve(Get_INSCONFIG_NumTranslations(insconfigmem));
  for(size_t idx = 0;
             idx < Get_INSCONFIG_NumTranslations(insconfigmem);
             idx++)
  {
    const INSCONFIG_TranslationMem* trmem = Get_INSCONFIG_Translation(insconfigmem, idx);
    novatel_oem7_msgs::msg::Translation& tr = insconfig->translations[idx];

    tr.translation.type    = trmem->translation;
    tr.frame.frame         = trmem->frame;
    tr.x_offset            = trmem->x_offset;
    tr.y_offset            = trmem->y_offset;
    tr.z_offset            = trmem->z_offset;
    tr.x_uncertainty       = trmem->x_uncertainty;
    tr.y_uncertainty       = trmem->y_uncertainty;
    tr.z_uncertainty       = trmem->z_uncertainty;
    tr.translation_source.status  = trmem->translation_source;
  }

  insconfig->rotations.reserve(Get_INSCONFIG_NumRotations(insconfigmem));
  for(size_t idx = 0;
             idx < Get_INSCONFIG_NumRotations(insconfigmem);
             idx++)
  {
    const INSCONFIG_RotationMem* rtmem = Get_INSCONFIG_Rotation(insconfigmem, idx);
    novatel_oem7_msgs::msg::Rotation& rt = insconfig->rotations[idx];
    rt.rotation.offset         = rtmem->rotation;
    rt.frame.frame             = rtmem->frame;
    rt.x_rotation              = rtmem->x_rotation;
    rt.y_rotation              = rtmem->y_rotation;
    rt.z_rotation              = rtmem->z_rotation;
    rt.x_rotation_stdev        = rtmem->x_rotation_stdev;
    rt.y_rotation_stdev        = rtmem->y_rotation_stdev;
    rt.z_rotation_stdev        = rtmem->z_rotation_stdev;
    rt.rotation_source.status  = rtmem->rotation_source;
  }

  static const std::string name = "INSCONFIG";
  SetOem7Header(msg, name, insconfig->nov_header);
}


template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::INSPVAX>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::INSPVAX>& inspvax)
{
  assert(msg->getMessageId() == INSPVAX_OEM7_MSGID);

  const auto* mem = reinterpret_cast<const INSPVAXMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  inspvax.reset(new novatel_oem7_msgs::msg::INSPVAX);

  inspvax->ins_status.status        = mem->ins_status;
  inspvax->pos_type.type            = mem->pos_type;
  inspvax->latitude                 = mem->latitude;
  inspvax->longitude                = mem->longitude;
  inspvax->height                   = mem->height;
  inspvax->undulation               = mem->undulation;
  inspvax->north_velocity           = mem->north_velocity;
  inspvax->east_velocity            = mem->east_velocity;
  inspvax->up_velocity              = mem->up_velocity;
  inspvax->roll                     = mem->roll;
  inspvax->pitch                    = mem->pitch;
  inspvax->azimuth                  = mem->azimuth;
  inspvax->latitude_stdev           = mem->latitude_stdev;
  inspvax->longitude_stdev          = mem->longitude_stdev;
  inspvax->height_stdev             = mem->height_stdev;
  inspvax->north_velocity_stdev     = mem->north_velocity_stdev;
  inspvax->east_velocity_stdev      = mem->east_velocity_stdev;
  inspvax->up_velocity_stdev        = mem->up_velocity_stdev;
  inspvax->roll_stdev               = mem->roll_stdev;
  inspvax->pitch_stdev              = mem->pitch_stdev;
  inspvax->azimuth_stdev            = mem->azimuth_stdev;
  inspvax->ext_sol_status.status    = mem->extended_status;

  static const std::string name = "INSPVAX";
  SetOem7Header(msg, name, inspvax->nov_header);
}



template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::INSSTDEV>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::INSSTDEV>& insstdev)
{
  assert(msg->getMessageId() == INSSTDEV_OEM7_MSGID);

  const auto* raw = reinterpret_cast<const INSSTDEVMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  insstdev.reset(new novatel_oem7_msgs::msg::INSSTDEV);

  insstdev->latitude_stdev         = raw->latitude_stdev;
  insstdev->longitude_stdev        = raw->longitude_stdev;
  insstdev->height_stdev           = raw->height_stdev;
  insstdev->north_velocity_stdev   = raw->north_velocity_stdev;
  insstdev->east_velocity_stdev    = raw->east_velocity_stdev;
  insstdev->up_velocity_stdev      = raw->up_velocity_stdev;
  insstdev->roll_stdev             = raw->roll_stdev;
  insstdev->pitch_stdev            = raw->pitch_stdev;
  insstdev->azimuth_stdev          = raw->azimuth_stdev;
  insstdev->ext_sol_status.status  = raw->ext_sol_status;
  insstdev->time_since_last_update = raw->time_since_last_update;
  insstdev->reserved1              = raw->reserved1;
  insstdev->reserved2              = raw->reserved2;
  insstdev->reserved3              = raw->reserved3;

  static const std::string name = "INSSTDEV";
  SetOem7Header(msg, name, insstdev->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::CORRIMU>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::CORRIMU>& corrimu)
{
  corrimu.reset(new novatel_oem7_msgs::msg::CORRIMU);

  if(msg->getMessageId() == CORRIMUS_OEM7_MSGID)
  {
    const auto* raw = reinterpret_cast<const CORRIMUSMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
    corrimu->imu_data_count   = raw->imu_data_count;
    corrimu->pitch_rate       = raw->pitch_rate;
    corrimu->roll_rate        = raw->roll_rate;
    corrimu->yaw_rate         = raw->yaw_rate;
    corrimu->lateral_acc      = raw->lateral_acc;
    corrimu->longitudinal_acc = raw->longitudinal_acc;
    corrimu->vertical_acc     = raw->vertical_acc;
  }
  else if(msg->getMessageId() == IMURATECORRIMUS_OEM7_MSGID)
  {
    const auto* raw =
        reinterpret_cast<const IMURATECORRIMUSMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
    corrimu->imu_data_count   = 1; 
    corrimu->pitch_rate       = raw->pitch_rate;
    corrimu->roll_rate        = raw->roll_rate;
    corrimu->yaw_rate         = raw->yaw_rate;
    corrimu->lateral_acc      = raw->lateral_acc;
    corrimu->longitudinal_acc = raw->longitudinal_acc;
    corrimu->vertical_acc     = raw->vertical_acc;
  }
  else
  {
    assert(false);
  }

  static const std::string name = "CORRIMU";
  SetOem7ShortHeader(msg, name, corrimu->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::TIME>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::TIME>& time)
{
  assert(msg->getMessageId()== TIME_OEM7_MSGID);

  const auto* mem = reinterpret_cast<const TIMEMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  time.reset(new novatel_oem7_msgs::msg::TIME);

  time->clock_status  = mem->clock_status;
  time->offset        = mem->offset;
  time->offset_std    = mem->offset_std;
  time->utc_offset    = mem->utc_offset;
  time->utc_year      = mem->utc_year;
  time->utc_month     = mem->utc_month;
  time->utc_day       = mem->utc_day;
  time->utc_hour      = mem->utc_hour;
  time->utc_min       = mem->utc_min;
  time->utc_msec      = mem->utc_msec;
  time->utc_status    = mem->utc_status;

  static const std::string name = "TIME";
  SetOem7Header(msg, name, time->nov_header);
}

template<>
void
MakeROSMessage<novatel_oem7_msgs::msg::RXSTATUS>(
    const Oem7RawMessageIf::ConstPtr& msg,
    std::shared_ptr<novatel_oem7_msgs::msg::RXSTATUS>& rxstatus)
{
  assert(msg->getMessageId() == RXSTATUS_OEM7_MSGID);

  const auto* mem = reinterpret_cast<const RXSTATUSMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));
  rxstatus.reset(new novatel_oem7_msgs::msg::RXSTATUS);

  rxstatus->error              = mem->error;
  rxstatus->num_status_codes   = mem->num_status_codes;
  rxstatus->rxstat             = mem->rxstat;
  rxstatus->rxstat_pri_mask    = mem->rxstat_pri_mask;
  rxstatus->rxstat_set_mask    = mem->rxstat_set_mask;
  rxstatus->rxstat_clr_mask    = mem->rxstat_clr_mask;
  rxstatus->aux1_stat          = mem->aux1_stat;
  rxstatus->aux1_stat_pri      = mem->aux1_stat_pri;
  rxstatus->aux1_stat_set      = mem->aux1_stat_set;
  rxstatus->aux1_stat_clr      = mem->aux1_stat_clr;
  rxstatus->aux2_stat          = mem->aux2_stat;
  rxstatus->aux2_stat_pri      = mem->aux2_stat_pri;
  rxstatus->aux2_stat_set      = mem->aux2_stat_set;
  rxstatus->aux2_stat_clr      = mem->aux2_stat_clr;
  rxstatus->aux3_stat          = mem->aux3_stat;
  rxstatus->aux3_stat_pri      = mem->aux3_stat_pri;
  rxstatus->aux3_stat_set      = mem->aux3_stat_set;
  rxstatus->aux3_stat_clr      = mem->aux3_stat_clr;
  rxstatus->aux4_stat          = mem->aux4_stat;
  rxstatus->aux4_stat_pri      = mem->aux4_stat_pri;
  rxstatus->aux4_stat_set      = mem->aux4_stat_set;
  rxstatus->aux4_stat_clr      = mem->aux4_stat_clr;


  static const std::string name = "RXSTATUS";
  SetOem7Header(msg, name, rxstatus->nov_header);
};

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::RANGE>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::SBASALMANAC>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::CLOCKMODEL>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::DUALANTENNAHEADING>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::GALIONO>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::GALCLOCK>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::GALFNAVEPHEMERIS>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::GALINAVEPHEMERIS>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::GPSEPHEM>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::BESTPOS>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::BESTVEL>&);


template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::BESTUTM>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::INSPVA>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&, std::shared_ptr<novatel_oem7_msgs::msg::INSPVAX>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::INSCONFIG>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::INSSTDEV>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::CORRIMU>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::TIME>&);

template
void
MakeROSMessage(const Oem7RawMessageIf::ConstPtr&,  std::shared_ptr<novatel_oem7_msgs::msg::RXSTATUS>&);


//---------------------------------------------------------------------------------------------------------------
/***
 * Obtains DOP values from Oem7 PSRDOP2 message
 */
void
GetDOPFromPSRDOP2(
    const Oem7RawMessageIf::ConstPtr& msg,
    uint32_t system_to_use,
    double&      gdop,
    double&      pdop,
    double&      hdop,
    double&      vdop,
    double&      tdop)
{
  const auto* mem = reinterpret_cast<const PSRDOP2_FixedMem*>(msg->getMessageData(OEM7_BINARY_MSG_HDR_LEN));

  gdop  = mem->gdop;
  pdop  = mem->pdop;
  hdop  = mem->hdop;
  vdop  = mem->vdop;

  const size_t num_sys = Get_PSRDOP2_NumSystems(mem);

  for(int i = 0;
          i < num_sys;
          i++)
  {
    const PSRDOP2_SystemMem* sys = Get_PSRDOP2_System(mem, i);
    if(sys->system == system_to_use)
    {
      tdop = sys->tdop;
      break;
    }
  }
}





}
