/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
 */

#ifndef _PLAYERBOT_QUESTSPAWNINDEX_H
#define _PLAYERBOT_QUESTSPAWNINDEX_H

#include <unordered_map>
#include <vector>

#include "Define.h"
#include "TravelMgr.h"

// Maps `(mapId, RequiredNpcOrGo-style entry)` → list of spawn
// positions for that template on that map. The entry convention
// matches Quest::RequiredNpcOrGo: positive value = creature template
// id, negative value = gameobject template id (use the absolute
// value to look up in gameobject_template).
//
// Built once at module startup by scanning sObjectMgr's
// CreatureDataStore + GameObjectDataStore. Read-only thereafter.
//
// Used by the RPG do-quest action to walk directly to specific known
// spawns of a quest objective instead of wandering inside a POI
// cluster. Mirrors the reference's TravelMgr per-spawn destination
// indexing.
class QuestSpawnIndex
{
public:
    static QuestSpawnIndex* instance();

    // Build the index from sObjectMgr's spawn data. Safe to call
    // multiple times — second+ calls are no-ops. Call once after
    // sObjectMgr->LoadCreatures / LoadGameObjects have populated
    // their stores.
    void Init();

    // Returns spawns of `entry` on `mapId`. Empty list if none
    // indexed. Stable reference for the lifetime of the index.
    std::vector<WorldPosition> const& GetSpawns(uint32 mapId, int32 entry) const;

    [[nodiscard]] bool IsInitialized() const { return _initialized; }

private:
    QuestSpawnIndex() = default;

    bool _initialized{false};

    struct Key
    {
        uint32 mapId;
        int32 entry;
        bool operator==(Key const& o) const { return mapId == o.mapId && entry == o.entry; }
    };
    struct KeyHash
    {
        std::size_t operator()(Key const& k) const noexcept
        {
            return (std::size_t(k.mapId) << 32) ^ std::size_t(uint32(k.entry));
        }
    };

    std::unordered_map<Key, std::vector<WorldPosition>, KeyHash> _index;
    std::vector<WorldPosition> _empty;
};

#define sQuestSpawnIndex QuestSpawnIndex::instance()

#endif
