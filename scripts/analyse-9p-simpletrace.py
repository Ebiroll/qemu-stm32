#!/usr/bin/env python
# Pretty print 9p simpletrace log
# Usage: ./analyse-9p-simpletrace <trace-events> <trace-pid>
#
# Author: Harsh Prateek Bora

import simpletrace

class VirtFSRequestTracker(simpletrace.Analyzer):
	def begin(self):
		print "Pretty printing 9p simpletrace log ..."

        def complete_pdu(self, tag, id, err):
                print "ERROR (tag =", tag, ", id =", id, ",err =", err, ")"

        def v9fs_version(self, tag, id, msize, version):
                print "TVERSION (tag =", tag, ", msize =", msize, ", version =", version, ")"

        def v9fs_version_return(self, tag, id, msize, version):
                print "RVERSION (tag =", tag, ", msize =", msize, ", version =", version, ")"

        def v9fs_attach(self, tag, id, fid, afid, uname, aname):
                print "TATTACH (tag =", tag, ", fid =", fid, ", afid =", afid, ", uname =", uname, ", aname =", aname, ")"

	def v9fs_attach_return(self, tag, id, type, verison, path):
		print "RATTACH (tag =", tag, ", qid={type =", type, ", version =", version, ", path =", path, "})"

	def v9fs_stat(self, tag, id, fid):
		print "TSTAT (tag =", tag, ", fid =", fid, ")"

	def v9fs_stat_return(self, tag, id, mode, atime, mtime, length):
		print "RSTAT (tag =", tag, ", mode =", mode, ", atime =", atime, ", mtime =", mtime, ", length =", length, ")"

	def v9fs_getattr(self, tag, id, fid, request_mask):
		print "TGETATTR (tag =", tag, ", fid =", fid, ", request_mask =", hex(request_mask), ")"

	def v9fs_getattr_return(self, tag, id, result_mask, mode, uid, gid):
		print "RGETATTR (tag =", tag, ", result_mask =", hex(result_mask), ", mode =", oct(mode), ", uid =", uid, ", gid =", gid, ")"

	def v9fs_walk(self, tag, id, fid, newfid, nwnames):
		print "TWALK (tag =", tag, ", fid =", fid, ", newfid =", newfid, ", nwnames =", nwnames, ")"

	def v9fs_walk_return(self, tag, id, nwnames, qids):
		print "RWALK (tag =", tag, ", nwnames =", nwnames, ", qids =", hex(qids), ")"

	def v9fs_open(self, tag, id, fid, mode):
		print "TOPEN (tag =", tag, ", fid =", fid, ", mode =", oct(mode), ")"

	def v9fs_open_return(self, tag, id, type, version, path, iounit):
		print "ROPEN (tag =", tag,  ", qid={type =", type, ", version =", version, ", path =", path, "}, iounit =", iounit, ")"

	def v9fs_lcreate(self, tag, id, dfid, flags, mode, gid):
		print "TLCREATE (tag =", tag, ", dfid =", dfid, ", flags =", oct(flags), ", mode =", oct(mode), ", gid =", gid, ")"

	def v9fs_lcreate_return(self, id, type, version, path, iounit):
		print "RLCREATE (tag =", tag,  ", qid={type =", type, ", version =", version, ", path =", path, "}, iounit =", iounit, ")"

	def v9fs_fsync(self, tag, id, fid, datasync):
		print "TFSYNC (tag =", tag, ", fid =", fid, ", datasync =", datasync, ")"

	def v9fs_clunk(self, tag, id, fid):
		print "TCLUNK (tag =", tag, ", fid =", fid, ")"

	def v9fs_read(self, tag, id, fid, off, max_count):
		print "TREAD (tag =", tag, ", fid =", fid, ", off =", off, ", max_count =", max_count, ")"

	def v9fs_read_return(self, tag, id, count, err):
		print "RREAD (tag =", tag, ", count =", count, ", err =", err, ")"

	def v9fs_readdir(self, tag, id, fid, offset, max_count):
		print "TREADDIR (tag =", tag, ", fid =", fid, ", offset =", offset, ", max_count =", max_count, ")"

	def v9fs_readdir_return(self, tag, id, count, retval):
		print "RREADDIR (tag =", tag, ", count =", count, ", retval =", retval, ")"

	def v9fs_write(self, tag, id, fid, off, count, cnt):
		print "TWRITE (tag =", tag, ", fid =", fid, ", off =", off, ", count =", count, ", cnt =", cnt, ")"

	def v9fs_write_return(self, tag, id, total, err):
		print "RWRITE (tag =", tag, ", total =", total, ", err =", err, ")"

	def v9fs_create(self, tag, id, fid, perm, name, mode):
		print "TCREATE (tag =", tag, ", fid =", fid, ", perm =", oct(perm), ", name =", name, ", mode =", oct(mode), ")"

	def v9fs_create_return(self, tag, id, type, verison, path, iounit):
		print "RCREATE (tag =", tag,  ", qid={type =", type, ", version =", version, ", path =", path, "}, iounit =", iounit, ")"

	def v9fs_symlink(self, tag, id, fid, name, symname, gid):
		print "TSYMLINK (tag =", tag, ", fid =", fid, ", name =", name, ", symname =", symname, ", gid =", gid, ")"

	def v9fs_symlink_return(self, tag, id, type, version, path):
		print "RSYMLINK (tag =", tag,  ", qid={type =", type, ", version =", version, ", path =", path, "})"

	def v9fs_flush(self, tag, id, flush_tag):
		print "TFLUSH (tag =", tag, ", flush_tag =", flush_tag, ")"

	def v9fs_link(self, tag, id, dfid, oldfid, name):
		print "TLINK (tag =", tag, ", dfid =", dfid, ", oldfid =", oldfid, ", name =", name, ")"

	def v9fs_remove(self, tag, id, fid):
		print "TREMOVE (tag =", tag, ", fid =", fid, ")"

	def v9fs_wstat(self, tag, id, fid, mode, atime, mtime):
		print "TWSTAT (tag =", tag, ", fid =", fid, ", mode =", oct(mode), ", atime =", atime, "mtime =", mtime, ")"

	def v9fs_mknod(self, tag, id, fid, mode, major, minor):
		print "TMKNOD (tag =", tag, ", fid =", fid, ", mode =", oct(mode), ", major =", major, ", minor =", minor, ")"

	def v9fs_lock(self, tag, id, fid, type, start, length):
		print "TLOCK (tag =", tag, ", fid =", fid, "type =", type, ", start =", start, ", length =", length, ")"

	def v9fs_lock_return(self, tag, id, status):
		print "RLOCK (tag =", tag, ", status =", status, ")"

	def v9fs_getlock(self, tag, id, fid, type, start, length):
		print "TGETLOCK (tag =", tag, ", fid =", fid, "type =", type, ", start =", start, ", length =", length, ")"

	def v9fs_getlock_return(self, tag, id, type, start, length, proc_id):
		print "RGETLOCK (tag =", tag, "type =", type, ", start =", start, ", length =", length, ", proc_id =", proc_id,  ")"

	def v9fs_mkdir(self, tag, id, fid, name, mode, gid):
		print "TMKDIR (tag =", tag, ", fid =", fid, ", name =", name, ", mode =", mode, ", gid =", gid, ")"

	def v9fs_mkdir_return(self, tag, id, type, version, path, err):
		print "RMKDIR (tag =", tag,  ", qid={type =", type, ", version =", version, ", path =", path, "}, err =", err, ")"

	def v9fs_xattrwalk(self, tag, id, fid, newfid, name):
		print "TXATTRWALK (tag =", tag, ", fid =", fid, ", newfid =", newfid, ", xattr name =", name, ")"

	def v9fs_xattrwalk_return(self, tag, id, size):
		print "RXATTRWALK (tag =", tag, ", xattrsize  =", size, ")"

	def v9fs_xattrcreate(self, tag, id, fid, name, size, flags):
		print "TXATTRCREATE (tag =", tag, ", fid =", fid, ", name =", name, ", xattrsize =", size, ", flags =", flags, ")"

	def v9fs_readlink(self, tag, id, fid):
		print "TREADLINK (tag =", tag, ", fid =", fid, ")"

	def v9fs_readlink_return(self, tag, id, target):
		print "RREADLINK (tag =", tag, ", target =", target, ")"

simpletrace.run(VirtFSRequestTracker())
