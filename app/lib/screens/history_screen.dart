import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';

import '../models/history_record.dart';
import '../state/robot_controller.dart';

/// REQ-APP-04 (P1): 청소 완료 기록을 리스트로 조회.
class HistoryScreen extends StatefulWidget {
  const HistoryScreen({super.key});

  @override
  State<HistoryScreen> createState() => _HistoryScreenState();
}

class _HistoryScreenState extends State<HistoryScreen> {
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    await context.read<RobotController>().refreshHistory();
    if (mounted) setState(() => _loading = false);
  }

  @override
  Widget build(BuildContext context) {
    final dateFormat = DateFormat('yyyy-MM-dd HH:mm');

    return Scaffold(
      appBar: AppBar(title: const Text('청소 기록')),
      body: RefreshIndicator(
        onRefresh: _load,
        child: Consumer<RobotController>(
          builder: (context, controller, _) {
            if (_loading) {
              return const Center(child: CircularProgressIndicator());
            }
            final records = controller.history;
            if (records.isEmpty) {
              return const Center(child: Text('아직 청소 기록이 없습니다.'));
            }
            return ListView.separated(
              itemCount: records.length,
              separatorBuilder: (_, __) => const Divider(height: 1),
              itemBuilder: (context, index) =>
                  _HistoryTile(record: records[index], dateFormat: dateFormat),
            );
          },
        ),
      ),
    );
  }
}

class _HistoryTile extends StatelessWidget {
  const _HistoryTile({required this.record, required this.dateFormat});

  final HistoryRecord record;
  final DateFormat dateFormat;

  @override
  Widget build(BuildContext context) {
    return ListTile(
      leading: CircleAvatar(
        backgroundColor: Colors.blueAccent.withValues(alpha: 0.15),
        child: record.mapImageUrl != null
            ? ClipOval(
                child: Image.network(
                  record.mapImageUrl!,
                  fit: BoxFit.cover,
                  width: 40,
                  height: 40,
                  errorBuilder: (_, __, ___) =>
                      const Icon(Icons.map_outlined, color: Colors.blueAccent),
                ),
              )
            : const Icon(Icons.map_outlined, color: Colors.blueAccent),
      ),
      title: Text(dateFormat.format(record.finishedAt)),
      subtitle: Text(
        '소요 시간 ${record.durationLabel} · 장애물 ${record.obstaclesFound}건'
        '${record.note != null ? '\n${record.note}' : ''}',
      ),
      isThreeLine: record.note != null,
    );
  }
}
