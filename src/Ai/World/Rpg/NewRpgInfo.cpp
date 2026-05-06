#include "NewRpgInfo.h"

#include <cmath>

#include "Timer.h"

void NewRpgInfo::ChangeToGoGrind(WorldPosition pos)
{
    Reset();
    data = GoGrind{pos};
}

void NewRpgInfo::ChangeToGoCamp(WorldPosition pos)
{
    Reset();
    data = GoCamp{pos};
}

void NewRpgInfo::ChangeToWanderNpc()
{
    Reset();
    data = WanderNpc{};
}

void NewRpgInfo::ChangeToWanderRandom()
{
    Reset();
    data = WanderRandom{};
}

void NewRpgInfo::ChangeToDoQuest(uint32 questId, const Quest* quest)
{
    Reset();
    DoQuest do_quest;
    do_quest.questId = questId;
    do_quest.quest = quest;
    data = do_quest;
}

void NewRpgInfo::ChangeToTravelFlight(uint32 flightMasterEntry, WorldPosition flightMasterPos, std::vector<uint32> path)
{
    Reset();
    TravelFlight flight;
    flight.flightMasterEntry = flightMasterEntry;
    flight.flightMasterPos = flightMasterPos;
    flight.path = std::move(path);
    flight.inFlight = false;
    data = flight;
}

void NewRpgInfo::ChangeToOutdoorPvp(ObjectGuid::LowType capturePointSpawnId)
{
    startT = getMSTime();
    OutdoorPvP pvp;
    pvp.capturePointSpawnId = capturePointSpawnId;
    data = pvp;
}

void NewRpgInfo::ChangeToRest()
{
    Reset();
    data = Rest{};
}

void NewRpgInfo::ChangeToIdle()
{
    Reset();
    data = Idle{};
}

bool NewRpgInfo::CanChangeTo(NewRpgStatus)
{
    return true;
}

void NewRpgInfo::Reset()
{
    data = Idle{};
    startT = getMSTime();
    ClearTravel();
    // recentMoveFarAttempts is intentionally NOT cleared. Reset() runs
    // on every state change (ChangeToDoQuest, ChangeToIdle, etc.) and
    // the do-quest action oscillates through transitions during a
    // failure cycle — wiping the deque here would prevent the
    // MoveFarTo loop-breaker (nF >= 3 AND mF >= 3 → bothExhausted)
    // from converging. CountRecentAttempts already filters by
    // destination (within 10y), so stale entries for previous quests
    // don't affect new ones.
}

void NewRpgInfo::RecordMoveFarAttempt(WorldPosition const& dest, bool wasNodeTravel)
{
    // Cap at 6 (3 node + 3 mmap). The loop-breaker in MoveFarTo
    // requires nF >= 3 AND mF >= 3 to declare bothExhausted. Each
    // MoveFarTo failure cycle records BOTH a node attempt and a mmap
    // attempt, so a single 3-cap deque would pop the older type
    // before its count reached 3, structurally preventing
    // bothExhausted from triggering.
    if (recentMoveFarAttempts.size() >= 6)
        recentMoveFarAttempts.pop_front();
    MoveFarAttempt a;
    a.dest = dest;
    a.wasNodeTravel = wasNodeTravel;
    a.timestamp = getMSTime();
    recentMoveFarAttempts.push_back(a);
}

int NewRpgInfo::CountRecentAttempts(WorldPosition const& dest, bool wasNodeTravel) const
{
    int count = 0;
    for (auto const& a : recentMoveFarAttempts)
    {
        if (a.wasNodeTravel != wasNodeTravel)
            continue;
        // Treat destinations within 10y as "same dest" — small jitter
        // from quest objective re-resolution shouldn't reset the loop
        // detector.
        if (a.dest.GetMapId() != dest.GetMapId())
            continue;
        if (a.dest.GetExactDist2dSq(&dest) > 10.0f * 10.0f)
            continue;
        ++count;
    }
    return count;
}

NewRpgStatus NewRpgInfo::GetStatus()
{
    return std::visit([](auto&& arg) -> NewRpgStatus {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, Idle>) return RPG_IDLE;
        if constexpr (std::is_same_v<T, GoGrind>) return RPG_GO_GRIND;
        if constexpr (std::is_same_v<T, GoCamp>) return RPG_GO_CAMP;
        if constexpr (std::is_same_v<T, WanderNpc>) return RPG_WANDER_NPC;
        if constexpr (std::is_same_v<T, WanderRandom>) return RPG_WANDER_RANDOM;
        if constexpr (std::is_same_v<T, Rest>) return RPG_REST;
        if constexpr (std::is_same_v<T, DoQuest>) return RPG_DO_QUEST;
        if constexpr (std::is_same_v<T, TravelFlight>) return RPG_TRAVEL_FLIGHT;
        if constexpr (std::is_same_v<T, OutdoorPvP>) return RPG_OUTDOOR_PVP;
        return RPG_IDLE;
    }, data);
}

std::string NewRpgInfo::ToString()
{
    std::stringstream out;
    out << "Status: ";
    std::visit([&out, this](auto&& arg)
    {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, GoGrind>)
        {
            out << "GO_GRIND";
            out << "\nGrindPos: " << arg.pos.GetMapId() << " " << arg.pos.GetPositionX() << " "
                << arg.pos.GetPositionY() << " " << arg.pos.GetPositionZ();
            out << "\nlastGoGrind: " << startT;
        }
        else if constexpr (std::is_same_v<T, GoCamp>)
        {
            out << "GO_CAMP";
            out << "\nCampPos: " << arg.pos.GetMapId() << " " << arg.pos.GetPositionX() << " "
                << arg.pos.GetPositionY() << " " << arg.pos.GetPositionZ();
            out << "\nlastGoCamp: " << startT;
        }
        else if constexpr (std::is_same_v<T, WanderNpc>)
        {
            out << "WANDER_NPC";
            out << "\nnpcOrGoEntry: " << arg.npcOrGo.GetCounter();
            out << "\nlastWanderNpc: " << startT;
            out << "\nlastReachNpcOrGo: " << arg.lastReach;
        }
        else if constexpr (std::is_same_v<T, WanderRandom>)
        {
            out << "WANDER_RANDOM";
            out << "\nlastWanderRandom: " << startT;
        }
        else if constexpr (std::is_same_v<T, Idle>)
        {
            out << "IDLE";
        }
        else if constexpr (std::is_same_v<T, Rest>)
        {
            out << "REST";
            out << "\nlastRest: " << startT;
        }
        else if constexpr (std::is_same_v<T, DoQuest>)
        {
            out << "DO_QUEST";
            out << "\nquestId: " << arg.questId;
            out << "\nobjectiveIdx: " << arg.objectiveIdx;
            out << "\npoiPos: " << arg.pos.GetMapId() << " " << arg.pos.GetPositionX() << " "
                << arg.pos.GetPositionY() << " " << arg.pos.GetPositionZ();
            out << "\nlastReachPOI: " << (arg.lastReachPOI ? GetMSTimeDiffToNow(arg.lastReachPOI) : 0);
        }
        else if constexpr (std::is_same_v<T, TravelFlight>)
        {
            out << "TRAVEL_FLIGHT";
            out << "\nflightMasterEntry: " << arg.flightMasterEntry;
            out << "\nfromNode: " << arg.path[0];
            out << "\ntoNode: " << arg.path[arg.path.size() - 1];
            out << "\ninFlight: " << arg.inFlight;
        }
        else if constexpr (std::is_same_v<T, OutdoorPvP>)
        {
            out << "OUTDOOR_PVP";
            if (!arg.capturePointSpawnId)
                out << "\nNo capture point assigned.";
            else
                out << "\ncapturePointSpawnId: " << arg.capturePointSpawnId;
        }
        else
            out << "UNKNOWN";
    }, data);
    return out.str();
}
