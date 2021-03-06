/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2012-2013 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#include "util_capture.h"
#include <string>

class UtilTrace {
public:
    UtilTrace(const pxcCHAR *task_name, PXCSessionService *ss) { this->ss=ss; if (ss) ss->TraceBegin(task_name); }
    ~UtilTrace() { if (ss) ss->TraceEnd(); }
protected:
    PXCSessionService *ss;
};

bool UtilCapture::RecordProperties(std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs) {
    for (std::map<PXCCapture::Device::Property,pxcF32>::iterator itrp=m_devcap_filters.begin();itrp!=m_devcap_filters.end();itrp++)
        m_device->SetProperty(itrp->first,itrp->second);

    /* The device must be able to provide all requested properties */
    for (std::vector<PXCCapture::VideoStream::DataDesc*>::iterator input=vinputs.begin();input!=vinputs.end();input++) {
        for (int i=0;i<PXCCapture::VideoStream::DEVCAP_LIMIT;i++) {
            if (!(*input)->devCaps[i].label) break;
            int label=(*input)->devCaps[i].label;
            pxcStatus sts=m_device->QueryProperty((PXCCapture::Device::Property)(label>0?label:-label),&(*input)->devCaps[i].value);
            if (sts<PXC_STATUS_NO_ERROR && label>0) return false;
        }
    }
    return true;
}

void UtilCapture::ScanProfiles(std::list<PXCCapture::VideoStream::ProfileInfo> &profiles, PXCCapture::VideoStream *vstream) {
    for (int profile_idx=0;;profile_idx++) {
        PXCCapture::VideoStream::ProfileInfo pinfo;
        pxcStatus sts=vstream->QueryProfile(profile_idx,&pinfo);
        if (sts<PXC_STATUS_NO_ERROR) break;

        if (m_vstream_filters.empty()) {
            profiles.push_back(pinfo);
		} else {
			bool filtered=false, ok=false;
            for (std::list<PXCCapture::VideoStream::ProfileInfo>::iterator vs=m_vstream_filters.begin();vs!=m_vstream_filters.end();vs++) {
				if ((vs->imageInfo.format&PXCImage::IMAGE_TYPE_MASK)!=(pinfo.imageInfo.format&PXCImage::IMAGE_TYPE_MASK)) continue;
				filtered=true;
				if (vs->imageInfo.width && vs->imageInfo.width!=pinfo.imageInfo.width) continue;
				if (vs->imageInfo.height && vs->imageInfo.height!=pinfo.imageInfo.height) continue;
				if (vs->frameRateMin.denominator) {
					float fmin=(float)vs->frameRateMin.numerator/vs->frameRateMin.denominator;
					if (pinfo.frameRateMin.denominator)	if (fmin>(float)pinfo.frameRateMin.numerator/pinfo.frameRateMin.denominator) continue;
					if (pinfo.frameRateMax.denominator)	if (fmin>(float)pinfo.frameRateMax.numerator/pinfo.frameRateMax.denominator) continue;
				}
				if (vs->frameRateMax.denominator) {
					float fmax=(float)vs->frameRateMax.numerator/vs->frameRateMax.denominator;
					if (pinfo.frameRateMin.denominator)	if (fmax<(float)pinfo.frameRateMin.numerator/pinfo.frameRateMin.denominator) continue;
					if (pinfo.frameRateMax.denominator)	if (fmax<(float)pinfo.frameRateMax.numerator/pinfo.frameRateMax.denominator) continue;
				}
				ok=true;
				break;
            }
            if (!filtered || ok) profiles.push_back(pinfo);
        }
    }
}

int UtilCapture::MatchProfiles(std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs, PXCCapture::Device::StreamInfo &sinfo, std::list<PXCCapture::VideoStream::ProfileInfo> &profiles, int vstream) {
    int filled=0;
    for (size_t i=0;i<m_maps.size();i++) {
        for (size_t j=0;j<PXCCapture::VideoStream::STREAM_LIMIT;j++) {
            if (m_maps[i][j]>=0) continue;
            PXCCapture::VideoStream::DataDesc::StreamDesc &sdesc=vinputs[i]->streams[j];
            if (!sdesc.format) break;
            if (sinfo.imageType!=(sdesc.format&PXCImage::IMAGE_TYPE_MASK)) continue;

            size_t nprofiles=profiles.size();
            for (std::list<PXCCapture::VideoStream::ProfileInfo>::iterator p=profiles.begin();p!=profiles.end();) {
                bool erase=false;
                if ((sdesc.format&PXCImage::IMAGE_TYPE_MASK)==PXCImage::IMAGE_TYPE_DEPTH && sdesc.format!=p->imageInfo.format) erase=true;
                if (sdesc.sizeMin.width>0 && p->imageInfo.width<sdesc.sizeMin.width) erase=true;
                if (sdesc.sizeMin.height>0 && p->imageInfo.height<sdesc.sizeMin.height) erase=true;
                if (sdesc.sizeMax.width>0 && p->imageInfo.width>sdesc.sizeMax.width) erase=true;
                if (sdesc.sizeMax.height>0 && p->imageInfo.height>sdesc.sizeMax.height) erase=true;

                if (erase) {
                    nprofiles--;
                    if (filled==0 || nprofiles>0) p=profiles.erase(p);
                    else p++;
                } else {
                    p++;
                }
            }
            if (nprofiles==0) continue;

            m_maps[i][j]=vstream; 
            filled++;
        }
    }
    return filled;
}

void UtilCapture::ClearMaps(std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs) {
    m_maps.resize(vinputs.size());
    for (size_t i=0;i<vinputs.size();i++) {
        m_maps[i].resize(PXCCapture::VideoStream::STREAM_LIMIT);
		for (size_t j=0;j<PXCCapture::VideoStream::STREAM_LIMIT;j++)
			m_maps[i][j]= -1;
	}
}

void UtilCapture::FindBestProfile(std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs, std::list<PXCCapture::VideoStream::ProfileInfo> &profiles, int vstream) {
    PXCCapture::VideoStream::ProfileInfo &pinfo=(*profiles.begin());
    if ((pinfo.imageInfo.format&PXCImage::IMAGE_TYPE_MASK)!=PXCImage::IMAGE_TYPE_COLOR) return;
    // for color image, we try the exact match.
    for (size_t i=0;i<m_maps.size();i++) {
        for (size_t j=0;j<PXCCapture::VideoStream::STREAM_LIMIT;j++) {
            if (m_maps[i][j]!=vstream) continue;
            for (std::list<PXCCapture::VideoStream::ProfileInfo>::iterator p=profiles.begin();p!=profiles.end();p++) {
                if (vinputs[i]->streams[j].format!=p->imageInfo.format) continue;
                pinfo=(*p);
                return;
            }
        }
    }
}

int UtilCapture::CalculateNumFormats(std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs) {
    int n1=0;
    for (size_t i=0;i<vinputs.size();i++)
        for (size_t j=0;j<PXCCapture::VideoStream::STREAM_LIMIT;j++)
            if (vinputs[i] && vinputs[i]->streams[j].format) n1++; else break;
    return n1;
}

pxcStatus UtilCapture::CreateCapture(pxcU32 index, PXCCapture **capture) {
    PXCSession::ImplDesc desc2;
    pxcStatus sts=m_session->QueryImpl(&m_desc_filter, index, &desc2);
    if (sts<PXC_STATUS_NO_ERROR) return PXC_STATUS_ITEM_UNAVAILABLE;
    return m_session->CreateImpl<PXCCapture>(&desc2,capture);
}

template <class T> bool __inline Consolidate(T &v1, T &v2) {
	if (!v2) return true;
	if (v1 && v1!=v2) return false;
	v1=v2;
	return true;
}

bool UtilCapture::ConsolidateAudioRequests(std::vector<PXCCapture::AudioStream::DataDesc*> &ainputs,PXCCapture::AudioStream::DataDesc *ainput) {
	memset(ainput,0,sizeof(*ainput));
	for (int i=0;i<(int)ainputs.size();i++) {
		if (!Consolidate<PXCAudio::AudioFormat>(ainput->info.format,ainputs[i]->info.format)) return false;
		if (!Consolidate<pxcU32>(ainput->info.nchannels,ainputs[i]->info.nchannels)) return false;
		if (!Consolidate<pxcU32>(ainput->info.bufferSize,ainputs[i]->info.bufferSize)) return false;
		if (!Consolidate<pxcU32>(ainput->info.sampleRate,ainputs[i]->info.sampleRate)) return false;
		if (!Consolidate<PXCAudio::ChannelMask>(ainput->info.channelMask,ainputs[i]->info.channelMask)) return false;
	}
	return true;
}

pxcStatus UtilCapture::LocateStreams(std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs,std::vector<PXCCapture::AudioStream::DataDesc*> &ainputs) {
    UtilTrace trace(L"UtilCapture::LocateStreams(video)", m_session_service);
	if (vinputs.empty() && ainputs.empty()) return PXC_STATUS_ITEM_UNAVAILABLE;

	PXCCapture::AudioStream::DataDesc ainput;
	if (!ConsolidateAudioRequests(ainputs,&ainput)) return PXC_STATUS_ITEM_UNAVAILABLE;

    int required_formats=CalculateNumFormats(vinputs);

    pxcStatus sts;
    m_desc_filter.group=PXCSession::IMPL_GROUP_SENSOR;
    m_desc_filter.subgroup=(vinputs.size()>0?PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE:0)|(ainputs.size()>0?PXCSession::IMPL_SUBGROUP_AUDIO_CAPTURE:0);
    for (int module_idx=0;;module_idx++) {
        sts = CreateCapture(module_idx, m_capture.ReleaseRef());
        if (sts == PXC_STATUS_ITEM_UNAVAILABLE) break;
        if (sts < PXC_STATUS_NO_ERROR) continue;

		// Loop through all of the devices supported by the capture module to find one
		// that provides streams that match the required streams.
        PXCCapture::DeviceInfo dinfo;
        for (int device_idx=0;;device_idx++) {
            sts=m_capture->QueryDevice(device_idx,&dinfo);
            if (sts<PXC_STATUS_NO_ERROR) break;
            if (m_session_service) m_session_service->TraceParam(L"Locating stream(s) on device ", dinfo.name);
            if (m_name_filter) if (!wcsstr(dinfo.name,m_name_filter)) continue;

            sts=m_capture->CreateDevice(device_idx,m_device.ReleaseRef());
            if (sts<PXC_STATUS_NO_ERROR) continue;

            /* Match image formats */
            ClearMaps(vinputs);

            int total_matched=0;
            bool am=(ainputs.size()>0)?false:true;

            for (int stream_idx=0;;stream_idx++) {
                PXCCapture::Device::StreamInfo sinfo;
                sts=m_device->QueryStream(stream_idx, &sinfo);
                if (sts<PXC_STATUS_NO_ERROR) break;

				// If this stream is a video stream (audio is checked below)
				// and there are still more streams that need to be matched then
				// check to see if this stream matches what is required.
				if (sinfo.cuid==PXCCapture::VideoStream::CUID && total_matched<required_formats) {
					PXCSmartPtr<PXCCapture::VideoStream> vstream;
					sts=m_device->CreateStream(stream_idx,PXCCapture::VideoStream::CUID,(void**)&vstream);
					if (sts<PXC_STATUS_NO_ERROR) break;

					std::list<PXCCapture::VideoStream::ProfileInfo> profiles;
					ScanProfiles(profiles,vstream);

					// Each stream has its own set of profiles. Check the streams to see how many
					// match the required profile.
					int num_matched=MatchProfiles(vinputs,sinfo,profiles,(int)m_vstreams.size());
					if (num_matched==0) continue;

					// Once the profiles are matched find the best one and set if for the stream
					// that will be used for this capture.
					FindBestProfile(vinputs,profiles,(int)m_vstreams.size());
					sts=vstream->SetProfile(&*profiles.begin());
					if (sts<PXC_STATUS_NO_ERROR) break;

					m_vstreams.push_back(vstream.ReleasePtr());
					total_matched+=num_matched;
				}
				if (sinfo.cuid==PXCCapture::AudioStream::CUID && !am) {
					sts=m_device->CreateStream(stream_idx, PXCCapture::AudioStream::CUID, (void**)m_astream.ReleaseRef());
					if (sts<PXC_STATUS_NO_ERROR) continue;

					for (int profile_idx=0;;profile_idx++) {
						PXCCapture::AudioStream::ProfileInfo pinfo;
						sts=m_astream->QueryProfile(profile_idx,&pinfo);
						if (sts<PXC_STATUS_NO_ERROR) break;

						if (ainput.info.nchannels>0  && ainput.info.nchannels!=pinfo.audioInfo.nchannels) continue;
						if (ainput.info.sampleRate>0 && ainput.info.sampleRate!=pinfo.audioInfo.sampleRate) continue;
						if (ainput.info.bufferSize>0 && ainput.info.bufferSize!=pinfo.audioInfo.bufferSize) continue;
						if (ainput.info.channelMask>0 && ainput.info.channelMask!=pinfo.audioInfo.channelMask) continue;

						sts=m_astream->SetProfile(&pinfo);
						if (sts<PXC_STATUS_NO_ERROR) break;

						for (size_t i=0;i<ainputs.size();i++) {
							memcpy_s(&ainputs[i]->info,sizeof(ainputs[i]->info),&pinfo.audioInfo,sizeof(pinfo.audioInfo));
							ainputs[i]->options=pinfo.audioOptions;
						}
                        am=true;
						break;
					}
					if (sts<PXC_STATUS_NO_ERROR) m_astream.ReleaseRef();
				}
				if (sts>=PXC_STATUS_NO_ERROR && total_matched>=required_formats && am) break;
            }
            if (sts>=PXC_STATUS_NO_ERROR && total_matched>=required_formats && am) 
                if (RecordProperties(vinputs)) break;

			// Unable to match all of the stream profiles for what was required so delete
			// whatever streams we had selected up to this point and release the device
			// so that the loop can check the next device.
            DeleteStreams();
            m_device.ReleaseRef();
        }
        if (sts>=PXC_STATUS_NO_ERROR)
        {
            if (m_session_service) m_session_service->TraceParam(L"Successfully located streams on device ", dinfo.name);
            // update actual image size
            for (size_t i = 0; i < vinputs.size() && i < m_maps.size(); i++)
            {
                for (size_t c = 0; c < PXCCapture::VideoStream::STREAM_LIMIT; c++)
                {
                    PXCCapture::VideoStream::ProfileInfo info;
            	    if (m_maps[i][c]<0) break;
                    PXCCapture::VideoStream* vstream = m_vstreams[m_maps[i][c]];
                    if (!vstream) break;
                    if (vstream->QueryProfile(&info) >= PXC_STATUS_NO_ERROR)
                    {
                        vinputs[i]->streams[c].sizeMin.width = info.imageInfo.width;
                        vinputs[i]->streams[c].sizeMin.height = info.imageInfo.height;
                        vinputs[i]->streams[c].sizeMax.width = info.imageInfo.width;
                        vinputs[i]->streams[c].sizeMax.height = info.imageInfo.height;
                    }
                }
            }
            break;
        }
        m_capture.ReleaseRef();
    }
    return sts;
}

pxcStatus UtilCapture::LocateStreams(PXCCapture::AudioStream::DataDesc *ainput) {
	std::vector<PXCCapture::VideoStream::DataDesc*> vinputs;
	std::vector<PXCCapture::AudioStream::DataDesc*> ainputs;
	ainputs.push_back(ainput);
	return LocateStreams(vinputs,ainputs);
}

pxcStatus UtilCapture::LocateStreams(PXCCapture::VideoStream::DataDesc* vinput) {
	std::vector<PXCCapture::VideoStream::DataDesc*> vinputs;
	std::vector<PXCCapture::AudioStream::DataDesc*> ainputs;
	vinputs.push_back(vinput);
	return LocateStreams(vinputs,ainputs);
}

pxcStatus UtilCapture::LocateStreams(std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs) {
	std::vector<PXCCapture::AudioStream::DataDesc*> ainputs;
	return LocateStreams(vinputs,ainputs);
}

pxcStatus UtilCapture::LocateStreams(std::vector<PXCCapture::AudioStream::DataDesc*> &ainputs) {
	std::vector<PXCCapture::VideoStream::DataDesc*> vinputs;
	return LocateStreams(vinputs,ainputs);
}

pxcStatus UtilCapture::ReadStreamAsync(PXCImage *images[], PXCScheduler::SyncPoint **sp) {
    UtilTrace trace(L"UtilCapture::ReadStreamAsync(video)", m_session_service);
	pxcU32 nvstreams=(pxcU32)m_vstreams.size();
    if (m_maps.size()==1) {   // for a single I/O, we use the m_maps order.
        if (nvstreams==1)
            return m_vstreams.front()->ReadStreamAsync(images,sp);

        PXCSmartSPArray spts(PXCCapture::VideoStream::STREAM_LIMIT);
        for (pxcU32 s=0;s<nvstreams;s++) {
            pxcStatus sts=m_vstreams[m_maps[0][s]]->ReadStreamAsync(&images[s],&spts[s]);
            if (sts<PXC_STATUS_NO_ERROR) return sts;
        }
        return PXCSmartAsyncImplIxOy<UtilCapture,PXCImage,PXCCapture::VideoStream::STREAM_LIMIT,PXCScheduler::SyncPoint,PXCCapture::VideoStream::STREAM_LIMIT>::
                SubmitTask(images,nvstreams,spts.ReleasePtrs(),nvstreams,sp,this,m_scheduler,&UtilCapture::ReadStreamSync,&UtilCapture::PassOnStatus,L"UtilCapture::ReadStreamSync");
    } else {    // for multiple I/O, we use the m_vstreams order. Each I/O is supposed to use MapStreams to get the right view.
        PXCSmartSPArray spts(PXCCapture::VideoStream::STREAM_LIMIT);
        for (pxcU32 s1=0;s1<nvstreams;s1++) {
            pxcStatus sts=m_vstreams[s1]->ReadStreamAsync(&images[s1],&spts[s1]);
            if (sts<PXC_STATUS_NO_ERROR) return sts;
        }
        return PXCSmartAsyncImplIxOy<UtilCapture,PXCImage,PXCCapture::VideoStream::STREAM_LIMIT,PXCScheduler::SyncPoint,PXCCapture::VideoStream::STREAM_LIMIT>::
                SubmitTask(images,nvstreams,spts.ReleasePtrs(),nvstreams,sp,this,m_scheduler,&UtilCapture::ReadStreamSync,&UtilCapture::PassOnStatus,L"UtilCapture::ReadStreamSync");
    }
}

pxcStatus UtilCapture::ReadStreamSync(PXCImage *[], PXCScheduler::SyncPoint *spts[]) {
	pxcU32 nvstreams=(pxcU32)m_vstreams.size();
	if (spts[0]) spts[0]->SynchronizeEx(nvstreams,spts);
	for (pxcU32 i=0;i<nvstreams;i++) if (spts[i]) spts[i]->Release();
	return PXC_STATUS_NO_ERROR;
}

pxcStatus UtilCapture::ReadStreamAsync(PXCAudio **audio, PXCScheduler::SyncPoint **sp) {
    UtilTrace trace(L"UtilCapture::ReadStreamAsync(audio)", m_session_service);
    if (!m_astream) return PXC_STATUS_DEVICE_FAILED;
    return m_astream->ReadStreamAsync(audio,sp);
}

UtilCapture::UtilCapture(PXCSession *session) {
    m_session=session;
    m_session_service = session->DynamicCast<PXCSessionService>();
    session->CreateScheduler(&m_scheduler);
    memset(&m_desc_filter,0,sizeof(m_desc_filter));
    m_name_filter=0;
}

UtilCapture::~UtilCapture(void) {
    DeleteStreams();
}

void UtilCapture::DeleteStreams(void) {
    for (std::vector<PXCCapture::VideoStream*>::iterator s=m_vstreams.begin();s!=m_vstreams.end();s++)
        (*s)->Release();
    m_vstreams.clear();
    m_astream.ReleaseRef();
}

pxcStatus UtilCapture::MapImages(int input, PXCImage *in[], PXCImage *out[]) {
    if (m_maps.size()==1) // for a single I/O, images already reordered
    {
        int nvstreams=(int)m_vstreams.size();
        for (int s=0;s<nvstreams;s++) out[s] = in[s]; 
        return PXC_STATUS_NO_ERROR;
    }
    for (size_t i=0;i<(size_t)PXCCapture::VideoStream::STREAM_LIMIT;i++) {
        if (m_maps[(size_t)input][i]<0) break;
        out[i]=in[m_maps[(size_t)input][i]];
    }
    return PXC_STATUS_NO_ERROR;
}

PXCImage *UtilCapture::QueryImage(PXCImage *images[], PXCImage::ImageType type) {
    for (size_t i=0;i<m_vstreams.size();i++) {
        PXCImage::ImageInfo info;
        images[i]->QueryInfo(&info);
        if ((info.format&PXCImage::IMAGE_TYPE_MASK)==(type&PXCImage::IMAGE_TYPE_MASK)) return images[i];
    }
    return NULL;
}

PXCCapture::VideoStream *UtilCapture::QueryVideoStream(int channel, int input) {
	if (input>=(int)m_maps.size()) return 0;
	if (channel>=PXCCapture::VideoStream::STREAM_LIMIT) return 0;
	if (m_maps[(size_t)input][(size_t)channel]<0) return 0;
	return m_vstreams[m_maps[(size_t)input][channel]];
}

void UtilCapture::SetFilter(PXCImage::ImageType type, PXCSizeU32 &size, pxcU32 fps) {
	PXCCapture::VideoStream::ProfileInfo pinfo;
	memset(&pinfo,0,sizeof(pinfo));
	pinfo.imageInfo.format=(PXCImage::ColorFormat)type;
	pinfo.imageInfo.width=size.width;
	pinfo.imageInfo.height=size.height;
	pinfo.frameRateMin.numerator=fps;
	pinfo.frameRateMin.denominator=fps?1:0;
	m_vstream_filters.push_back(pinfo);
}

